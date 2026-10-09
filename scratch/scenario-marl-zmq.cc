/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/* *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Andrea Lacava <thecave003@gmail.com>
 *          Michele Polese <michele.polese@gmail.com>
 *          Matteo Bordin <matbord97@gmail.com>
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include <ns3/lte-ue-net-device.h>
#include "ns3/mmwave-helper.h"
#include "ns3/channel-condition-model.h"
#include "ns3/three-gpp-propagation-loss-model.h"
#include "ns3/epc-helper.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"
#include "ns3/lte-helper.h"
#include "ns3/energy-heuristic.h"
#include "ns3/zmq-database-client.h"

using namespace ns3;
using namespace mmwave;

/**
 * Scenario Three
 * 
 */

NS_LOG_COMPONENT_DEFINE ("ScenarioThree");

std::ofstream outFile;

// ---------------------------------------------------------------------------
// Periodic MARL control step, scheduled every T_control seconds.
// ---------------------------------------------------------------------------
// Handover events for the PingPong reward term. One record per handover start,
// accumulated by the trace callback below and drained by MarlControlStep exactly
// once per control period.
struct MarlHandoverRecord
{
    double timeS;
    uint64_t imsi;
    uint16_t srcCell;
    uint16_t dstCell;
};
static std::vector<MarlHandoverRecord> g_marlHandovers;

// Set from controlPhaseOffsetS (see g_controlPhaseOffsetS). When true, the
// SINR bins come from the copy banked by the DU report build instead of the
// live counters, which that build has just reset.
static bool g_marlUseBankedSinrBins = false;

// Set from handoverSinrFilterTauMs > 0: also emit, per UE, the (filtered) SINR
// the RRC handover decision uses, as ho_decision_sinr_db. Diagnostic only.
static bool g_marlEmitHoDecisionSinr = false;

// The signature has to match the UE RRC HandoverStart trace exactly. See
// MmWaveBearerStatsConnector::NotifyHandoverStartUe.
void MarlHandoverStartCallback(std::string /*context*/,
                               uint64_t imsi,
                               uint16_t cellId,
                               uint16_t /*rnti*/,
                               uint16_t targetCellId)
{
    g_marlHandovers.push_back({ns3::Simulator::Now().GetSeconds(), imsi, cellId, targetCellId});
}

void MarlControlStep(ns3::ZmqDatabaseClient* zmqClient,
                     ns3::Ptr<ns3::LteEnbNetDevice> enbDevice,
                     ns3::NetDeviceContainer mmWaveEnbDevs,
                     double stepInterval)
{
    double now = ns3::Simulator::Now().GetSeconds();

    // 1. Package cell KPIs into clean JSON, reading the live counters that the
    // DU/CU-CP report builders (mmwave-enb-net-device.cc) already maintain for
    // every real NR cell, instead of a single hardcoded cell.
    json kpiPayload;
    kpiPayload["timestamp"] = now;

    for (uint32_t i = 0; i < mmWaveEnbDevs.GetN(); ++i)
    {
        ns3::Ptr<ns3::mmwave::MmWaveEnbNetDevice> mmDev =
            ns3::DynamicCast<ns3::mmwave::MmWaveEnbNetDevice>(mmWaveEnbDevs.Get(i));
        if (!mmDev)
        {
            continue;
        }

        uint16_t cellId = mmDev->GetCellId();
        std::string cellKey = std::to_string(cellId);

        auto ueMap = mmDev->GetRrc()->GetUeMap();
        auto l3SinrMap = mmDev->Getl3sinrMap();

        json& cellPayload = kpiPayload["cells"][cellKey];
        // DRB.RRU.PrbUsedDl / dlPrbUsage, as a [0,1] fraction
        cellPayload["prb_utilization"] = mmDev->GetDlPrbUsage() / 100.0;
        // DRB.BufferSize.Qos, RLC tx buffer occupancy summed over connected UEs (bytes)
        cellPayload["buffer_bytes"] = mmDev->GetRlcBufferOccupancyCellSpecific();
        // QosFlow.PdcpPduVolumeDL_Filter, MAC DL volume summed over connected UEs (bytes)
        cellPayload["volume_bytes"] = mmDev->GetMacVolumeCellSpecific();
        cellPayload["num_active_ues"] = static_cast<uint32_t>(ueMap.size());

        // DL PDCP delivered bytes, drained from this cell's own accumulator.
        // This is what the Satisfaction reward term consumes, not the per-UE
        // dl_pdcp_delivered_bytes below, which is a raw read of a counter whose
        // reset can be missed during handover. See the DrainMarlDlRxBytes docs.
        //
        // Emitted at cell level rather than inside "ues" on purpose: the drained
        // map can hold an IMSI that delivered bytes earlier in the period and has
        // since moved to another cell, so it is not a subset of the current
        // ueMap. Nesting it under "ues" would silently drop exactly those bytes.
        //
        // pdcp_window_s is the simulated time this batch covers. It is reported
        // rather than assumed, so Python never has to guess the window and the
        // value stays correct if T_control changes.
        double pdcpWindowS = 0.0;
        auto drainedPdcp = mmDev->DrainMarlDlRxBytes (pdcpWindowS);
        json deliveredPayload = json::object();
        for (auto& kv : drainedPdcp)
        {
            deliveredPayload[std::to_string (kv.first)] = kv.second;
        }
        cellPayload["pdcp_delivered_bytes"] = deliveredPayload;
        cellPayload["pdcp_window_s"] = pdcpWindowS;

        // L1M.RS-SINR bin distribution, 7 bins, summed over this cell's UEs.
        // Feeds the BadSignal reward term, which wants bins[0] / sum(bins) =
        // fraction of DL transmissions at <= -6 dB.
        //
        // Source counters: MmWavePhyTrace::m_macSinrBin1..7UeSpecific, written
        // by MmWavePhyTrace::UpdateTraces() (mmwave-phy-trace.cc:306-333),
        // which buckets every DL transmission by 10*log10(sinr) into
        //   [0] <= -6 dB, [1] <= 0, [2] <= 6, [3] <= 12, [4] <= 18, [5] <= 24, [6] > 24.
        //
        // Read through the same MmWavePhyTrace instance that already backs
        // volume_bytes and prb_utilization, the device's "E2DuCalculator"
        // attribute (mmwave-enb-net-device.cc:280-284). Two consequences worth
        // knowing. First, these bins are alive exactly when volume_bytes is,
        // since UpdateTraces() increments both on the same call. Second, they are
        // reset by ResetPhyTracesForRntiCellId (mmwave-phy-trace.cc:506-518)
        // alongside m_macVolumeUeSpecific, so they cover the same 0.1 s E2
        // indication window as volume_bytes, not the whole control period.
        ns3::PointerValue duCalcValue;
        mmDev->GetAttribute ("E2DuCalculator", duCalcValue);
        ns3::Ptr<ns3::mmwave::MmWavePhyTrace> duCalc =
            duCalcValue.Get<ns3::mmwave::MmWavePhyTrace> ();
        std::vector<uint32_t> sinrBins (7, 0);
        if (g_marlUseBankedSinrBins)
        {
            sinrBins = mmDev->GetMarlBankedSinrBins ();
        }
        else if (duCalc)
        {
            for (auto& ue : ueMap)
            {
                uint16_t rnti = ue.second->GetRnti ();
                sinrBins[0] += duCalc->GetMacSinrBin1UeSpecific (rnti, cellId);
                sinrBins[1] += duCalc->GetMacSinrBin2UeSpecific (rnti, cellId);
                sinrBins[2] += duCalc->GetMacSinrBin3UeSpecific (rnti, cellId);
                sinrBins[3] += duCalc->GetMacSinrBin4UeSpecific (rnti, cellId);
                sinrBins[4] += duCalc->GetMacSinrBin5UeSpecific (rnti, cellId);
                sinrBins[5] += duCalc->GetMacSinrBin6UeSpecific (rnti, cellId);
                sinrBins[6] += duCalc->GetMacSinrBin7UeSpecific (rnti, cellId);
            }
        }
        cellPayload["sinr_bins"] = sinrBins;

        // Per-UE DL PDCP bytes actually DELIVERED (received), feeding the
        // Satisfaction reward term.
        //
        // Source: MmWaveBearerStatsCalculator::GetDlRxData(imsi, LCID 3), a
        // pure read that returns m_dlRxData[p] with no side effect. It is the
        // same counter the CU-UP report builder turns into
        // DRB.UEThpDlPdcpBased.UEID / drbuethpdlpdcpbasedueid
        // (mmwave-enb-net-device.cc:748, becoming pdcpThroughputRx at :824).
        // Received, not transmitted, so it is not retransmission-inflated.
        // GetDlTxData and the RLC GetTxBytesInReportingPeriod are both TX and
        // both wrong for this term.
        //
        // We read the calculator rather than m_drbThrDlPdcpBasedComputationUeid
        // because that member is filled by the CU-UP builder (:829) and then read
        // and cleared by the DU builder (:1323, :1380) inside the same
        // BuildAndSendReportMessage call. Reports run at E2Periodicity offset by
        // 800 us (0.0008, 0.1008, ... scheduled at :615 and :1642) while
        // MarlControlStep runs on its own T_control grid starting at 0.0, so at
        // every control step that member has just been cleared and would read
        // deterministically zero. Reading the calculator sidesteps it entirely.
        //
        // We reset nothing here, so the CU-UP report at t+0.0008 still sees its
        // full window and every existing consumer stays byte-identical: the CU-UP
        // report, the DU report's drbThrDlPdcpBasedUeid, and the offline CSV/DB.
        // Our value covers the 99.2 ms since the last report's reset (:864)
        // rather than a full 100 ms, a deliberate ~0.8% undercount.
        //
        // If the calculator pointer does not resolve we emit no key at all, so
        // Python can tell "measured, and it was zero" (a real, penalisable
        // failure) from "not measured" (the term must be dropped, not zeroed).
        ns3::PointerValue pdcpCalcValue;
        mmDev->GetAttribute ("E2PdcpCalculator", pdcpCalcValue);
        ns3::Ptr<ns3::mmwave::MmWaveBearerStatsCalculator> pdcpCalc =
            pdcpCalcValue.Get<ns3::mmwave::MmWaveBearerStatsCalculator> ();

        json uesPayload = json::object();
        for (auto& ue : ueMap)
        {
            uint64_t imsi = ue.second->GetImsi();
            std::string imsiKey = std::to_string(imsi);

            // Written before the SINR lookups below on purpose: those `continue`
            // past any UE with no L3 SINR report, and a UE can be delivering data
            // while having no measurement report yet. Emitting PDCP first means
            // such a UE still reports its bytes instead of vanishing from the
            // payload and silently undercounting Satisfaction.
            if (pdcpCalc)
            {
                uesPayload[imsiKey]["dl_pdcp_delivered_bytes"] =
                    static_cast<double>(pdcpCalc->GetDlRxData(imsi, 3));
                // Sibling counter, emitted so delivered can be validated against
                // a quantity it cannot legitimately exceed. Same calculator
                // object, same (imsi, lcid=3) key, and erased by the same
                // ResetResultsForImsiLcid call that clears m_dlRxData
                // (mmwave-bearer-stats-calculator.cc), so tx and rx cover the
                // identical window by construction. rx <= tx is a hard invariant:
                // you cannot receive more than was sent, so rx/tx > 1 means the
                // delivered counter is wrong.
                uesPayload[imsiKey]["dl_pdcp_tx_bytes"] =
                    static_cast<double>(pdcpCalc->GetDlTxData(imsi, 3));
            }

            auto ueSinrIt = l3SinrMap.find(imsi);
            if (ueSinrIt == l3SinrMap.end())
            {
                continue;
            }
            auto sinrIt = ueSinrIt->second.find(cellId);
            if (sinrIt == ueSinrIt->second.end())
            {
                continue;
            }
            // L3 serving SINR, converted from linear to dB (mirrors
            // BuildRicIndicationMessageCuCp's sinrThisCell computation)
            uesPayload[imsiKey]["l3_serving_sinr_db"] = 10 * std::log10(sinrIt->second);
            if (g_marlEmitHoDecisionSinr)
            {
                // The SINR the RRC's handover decision uses for this UE and
                // cell (filtered when handoverSinrFilterTauMs > 0). The
                // agent-facing l3_serving_sinr_db above stays raw.
                double hoDb = enbDevice->GetRrc ()->GetHandoverDecisionSinrDb (imsi, cellId);
                if (std::isfinite (hoDb))
                {
                    uesPayload[imsiKey]["ho_decision_sinr_db"] = hoDb;
                }
            }
        }
        cellPayload["ues"] = uesPayload;
    }

    // 1b. Drain the handovers seen since the previous control step.
    // Top-level rather than per-cell: a handover belongs to a pair of cells, so
    // nesting it under one of them would force an arbitrary choice and make the
    // Python side re-derive the pairing. Emitted even when empty, so Python can
    // tell "no handovers this step" (a real, meaningful zero) from "this ns-3
    // build does not send the field at all".
    json handoverPayload = json::array();
    for (const auto& h : g_marlHandovers)
    {
        json rec;
        rec["t"] = h.timeS;
        rec["imsi"] = h.imsi;
        rec["src"] = h.srcCell;
        rec["dst"] = h.dstCell;
        handoverPayload.push_back(rec);
    }
    g_marlHandovers.clear();          // we own the fill-and-drain cycle
    kpiPayload["handovers"] = handoverPayload;
    kpiPayload["handover_window_s"] = stepInterval;

    // 2. Synchronous handshake: send KPIs and block until Python Gym replies
    json actionPayload = zmqClient->StepSync(kpiPayload);
    
    // 3. Safely apply the received actions back to ns-3 (CIO offsets for every real cell
    // present in actionPayload["cells"], applied via the shared LTE anchor RRC)
    if (actionPayload.is_object())
    {
        enbDevice->ApplyControlPayload(actionPayload);
    }

    // 4. Schedule the next control step
    ns3::Simulator::Schedule(ns3::Seconds(stepInterval),
                             &MarlControlStep,
                             zmqClient,
                             enbDevice,
                             mmWaveEnbDevs,
                             stepInterval);
}


void
BsStateTrace (std::string filename, Ptr<LteEnbNetDevice> ltedev, Ptr<LteEnbRrc> lte_rrc )
{
    if (!outFile.is_open ())
    {
      outFile.open (filename.c_str (), std::ios_base::out | std::ios_base::trunc);
      NS_LOG_LOGIC ("File opened");
      outFile << "Timestamp"
              << " "
              << "UNIX"
              << " "
              << "Id"
              << " "
              << "State" << std::endl;
    }
  std::map<uint16_t, bool> entry = lte_rrc->GetAllowHandoverTo();
  for (auto it = entry.begin(); it != entry.end(); it++)
  {
    uint64_t timestamp = ltedev->GetStartTime() + Simulator::Now ().GetMilliSeconds ();
    outFile << Simulator::Now ().GetSeconds () << " " << timestamp << " "
                          << it->first << " " << it->second << std::endl;
  }
}

void
PrintGnuplottableUeListToFile (std::string filename)
{
  std::ofstream outFile;
  outFile.open (filename.c_str (), std::ios_base::out | std::ios_base::trunc);
  if (!outFile.is_open ())
    {
      NS_LOG_ERROR ("Can't open file " << filename);
      return;
    }
  for (NodeList::Iterator it = NodeList::Begin (); it != NodeList::End (); ++it)
    {
      Ptr<Node> node = *it;
      int nDevs = node->GetNDevices ();
      for (int j = 0; j < nDevs; j++)
        {
          Ptr<LteUeNetDevice> uedev = node->GetDevice (j)->GetObject<LteUeNetDevice> ();
          Ptr<MmWaveUeNetDevice> mmuedev = node->GetDevice (j)->GetObject<MmWaveUeNetDevice> ();
          Ptr<McUeNetDevice> mcuedev = node->GetDevice (j)->GetObject<McUeNetDevice> ();
          if (uedev)
            {
              Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
              outFile << "set label \"" << uedev->GetImsi () << "\" at " << pos.x << "," << pos.y
                      << " left font \"Helvetica,8\" textcolor rgb \"black\" front point pt 1 ps "
                         "0.3 lc rgb \"black\" offset 0,0"
                      << std::endl;
            }
          else if (mmuedev)
            {
              Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
              outFile << "set label \"" << mmuedev->GetImsi () << "\" at " << pos.x << "," << pos.y
                      << " left font \"Helvetica,8\" textcolor rgb \"black\" front point pt 1 ps "
                         "0.3 lc rgb \"black\" offset 0,0"
                      << std::endl;
            }
          else if (mcuedev)
            {
              Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
              outFile << "set label \"" << mcuedev->GetImsi () << "\" at " << pos.x << "," << pos.y
                      << " left font \"Helvetica,8\" textcolor rgb \"black\" front point pt 1 ps "
                         "0.3 lc rgb \"black\" offset 0,0"
                      << std::endl;
            }
        }
    }
}

void
PrintGnuplottableEnbListToFile (std::string filename)
{
  std::ofstream outFile;
  outFile.open (filename.c_str (), std::ios_base::out | std::ios_base::trunc);
  if (!outFile.is_open ())
    {
      NS_LOG_ERROR ("Can't open file " << filename);
      return;
    }
  for (NodeList::Iterator it = NodeList::Begin (); it != NodeList::End (); ++it)
    {
      Ptr<Node> node = *it;
      int nDevs = node->GetNDevices ();
      for (int j = 0; j < nDevs; j++)
        {
          Ptr<LteEnbNetDevice> enbdev = node->GetDevice (j)->GetObject<LteEnbNetDevice> ();
          Ptr<MmWaveEnbNetDevice> mmdev = node->GetDevice (j)->GetObject<MmWaveEnbNetDevice> ();
          if (enbdev)
            {
              Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
              outFile << "set label \"" << enbdev->GetCellId () << "\" at " << pos.x << "," << pos.y
                      << " left font \"Helvetica,8\" textcolor rgb \"blue\" front  point pt 4 ps "
                         "0.3 lc rgb \"blue\" offset 0,0"
                      << std::endl;
            }
          else if (mmdev)
            {
              Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
              outFile << "set label \"" << mmdev->GetCellId () << "\" at " << pos.x << "," << pos.y
                      << " left font \"Helvetica,8\" textcolor rgb \"red\" front  point pt 4 ps "
                         "0.3 lc rgb \"red\" offset 0,0"
                      << std::endl;
            }
        }
    }
}

// logChannelConditions diagnostic: last LOS/NLOS state seen per (UE node, eNB
// node), and the number of changes observed. Queries the same channel
// condition model the path-loss model uses, so a link not yet evaluated by the
// simulation is created (one RNG draw) by the query itself: use only in
// diagnostic runs, never in a run whose payload must stay byte-identical.
static std::map<std::pair<uint32_t, uint32_t>, int> g_lastLosCondition;
static uint64_t g_losConditionChanges = 0;

void
LogChannelConditions (Ptr<ChannelConditionModel> condModel, NodeContainer ues,
                      NodeContainer enbs, double period, std::string filename)
{
  std::ofstream f (filename, std::ios_base::app);
  for (uint32_t u = 0; u < ues.GetN (); ++u)
    {
      for (uint32_t e = 0; e < enbs.GetN (); ++e)
        {
          Ptr<MobilityModel> a = ues.Get (u)->GetObject<MobilityModel> ();
          Ptr<MobilityModel> b = enbs.Get (e)->GetObject<MobilityModel> ();
          int c = static_cast<int> (condModel->GetChannelCondition (a, b)->GetLosCondition ());
          auto key = std::make_pair (ues.Get (u)->GetId (), enbs.Get (e)->GetId ());
          auto it = g_lastLosCondition.find (key);
          if (it == g_lastLosCondition.end () || it->second != c)
            {
              if (it != g_lastLosCondition.end ())
                {
                  g_losConditionChanges++;
                }
              g_lastLosCondition[key] = c;
              f << Simulator::Now ().GetSeconds () << " " << key.first << " " << key.second
                << " " << c << std::endl;
            }
        }
    }
  Simulator::Schedule (Seconds (period), &LogChannelConditions, condModel, ues, enbs, period,
                       filename);
}

void
PrintPosition (Ptr<Node> node)
{
  Ptr<MobilityModel> model = node->GetObject<MobilityModel> ();
  NS_LOG_UNCOND("Position +****************************** ID "
                << node->GetId() << " coords " << model->GetPosition() << " at time "
                << Simulator::Now().GetSeconds());
}

static ns3::GlobalValue g_bufferSize ("bufferSize", "RLC tx buffer size (MB)",
                                      ns3::UintegerValue (10),
                                      ns3::MakeUintegerChecker<uint32_t> ());

static ns3::GlobalValue g_rlcAmEnabled ("rlcAmEnabled", "If true, use RLC AM, else use RLC UM",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_enableTraces ("enableTraces", "If true, generate ns-3 traces",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());
                                                                                
static ns3::GlobalValue g_e2lteEnabled ("e2lteEnabled", "If true, send LTE E2 reports",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_e2nrEnabled ("e2nrEnabled", "If true, send NR E2 reports",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_e2du ("e2du", "If true, send DU reports",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_e2cuUp ("e2cuUp", "If true, send CU-UP reports",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_e2cuCp ("e2cuCp", "If true, send CU-CP reports",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

// ZeroMQ port the MARL bridge connects to. This used to be hardcoded to 5555,
// which made parallel training impossible: two ns-3 instances would fight over
// one socket, so every A/B pair and every load sweep had to run sequentially, at
// about 95 min per arm. The Python side already took a port (MlbZmqEnv passes
// zmq_port down to ZmqStateDatabase); only this literal was missing. The 5555
// default reproduces the previous behaviour.
static ns3::GlobalValue g_zmqPort (
    "zmqPort", "TCP port of the MARL ZeroMQ bridge (tcp://localhost:PORT)",
    ns3::UintegerValue (5555), ns3::MakeUintegerChecker<uint16_t> ());

// Control period T_control in seconds, i.e. how often the agent observes and
// acts. This used to be a hardcoded local. Wall-clock cost scales with simulated
// seconds rather than with the number of RL steps, so halving this doubles the RL
// steps per unit of compute. The tradeoff is that it also shortens the time each
// CIO has to take effect (DynamicTtt TTT is 25-150 ms), so it changes the control
// problem slightly. Keep Python's control_period_s consistent when changing it.
// The 1.0 default reproduces the previous behaviour.
static ns3::GlobalValue g_controlInterval (
    "controlInterval", "MARL control period T_control in seconds",
    ns3::DoubleValue (1.0), ns3::MakeDoubleChecker<double> (0.01));

// Phase of the MARL control step relative to the E2 report grid, in seconds.
//
// The DU/CU-UP report builders run at 0.0008, 0.1008, ... and are the only
// place prb_utilization, buffer_bytes, volume_bytes and the banked PDCP
// delivered bytes are refreshed. With the default 0.0 the step at k*T reads
// the report built 99.2 ms earlier, so every load KPI describes the window
// BEFORE the previous action (audit_oct_2.md I6). A value just above 0.0008
// (e.g. 0.001) runs the step right after the report, so step k reads the
// window ending at k*T + 0.0008.
//
// Side effects when > 0, both handled here:
//  - the live SINR-bin counters are reset by that same report build, so the
//    bins are taken from a copy banked there (MmWaveEnbNetDevice attribute
//    MarlBankSinrBins, switched on below);
//  - the per-UE raw dl_pdcp_delivered_bytes / dl_pdcp_tx_bytes are reset by
//    the CU-UP build too, so they then cover only the offset minus 0.8 ms.
//    The env's Satisfaction does not use them (it uses the banked
//    pdcp_delivered_bytes), but rx/tx validation scripts do.
// 0.0, the default, reproduces the previous schedule exactly.
// Secondary-cell handover hysteresis and TTT basis (LteEnbRrc attributes
// HandoverHysteresisDb / TttFromBiasedSinr, lte-enb-rrc.cc).
//
// Without hysteresis a UE is handed over whenever any cell's CIO-biased SINR
// edges past the serving cell's, so SINR fluctuation alone produced ~0.5
// handovers per UE per second with ~28% ping-pongs at zero CIO, about 100x
// the geometric boundary-crossing rate (audit_oct_3.md N). Both default to the
// previous behaviour, and the attributes are only touched when non-default.
static ns3::GlobalValue g_handoverHysteresisDb (
    "handoverHysteresisDb",
    "Handover margin (dB) on CIO-biased SINR for TTT-based secondary-cell handover. 0 = off.",
    ns3::DoubleValue (0.0), ns3::MakeDoubleChecker<double> (0.0, 20.0));
// UpdatePeriod of ns3::ThreeGppChannelConditionModel, in ms. With the
// historical 100 every UE-cell link re-draws LOS/NLOS independently every
// 100 ms (ComputeChannelCondition draws a fresh uniform against pLos), which
// keeps handovers flapping whatever filter or margin is used (audit_oct_5.md
// V.3). 0 = ns-3's own "never updated": each link's condition is computed once
// and kept. 100, the default, reproduces the previous behaviour exactly.
static ns3::GlobalValue g_channelConditionUpdatePeriodMs (
    "channelConditionUpdatePeriodMs",
    "UpdatePeriod (ms) of ThreeGppChannelConditionModel. 100 = previous behaviour; "
    "0 = compute each link's LOS/NLOS once and keep it.",
    ns3::UintegerValue (100), ns3::MakeUintegerChecker<uint32_t> ());
// Diagnostic only: log every change of each UE-to-mmWave-cell LOS/NLOS state to
// channel_conditions.txt, sampled every controlInterval. Perturbs the RNG (see
// LogChannelConditions). Off by default.
static ns3::GlobalValue g_logChannelConditions (
    "logChannelConditions",
    "Diagnostic: log UE-to-mmWave-cell LOS/NLOS state changes (perturbs the RNG).",
    ns3::BooleanValue (false), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_handoverSinrFilterTauMs (
    "handoverSinrFilterTauMs",
    "Time constant (ms) of the L3-style SINR filter feeding the handover decision "
    "(LteEnbRrc HandoverSinrFilterTauMs). 0 = off.",
    ns3::DoubleValue (0.0), ns3::MakeDoubleChecker<double> (0.0, 10000.0));
static ns3::GlobalValue g_tttFromBiasedSinr (
    "tttFromBiasedSinr",
    "DynamicTtt from the CIO-biased SINR difference instead of the raw one. false = off.",
    ns3::BooleanValue (false), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_controlPhaseOffsetS (
    "controlPhaseOffsetS",
    "Offset (s) of the MARL control step from the k*controlInterval grid. 0 = previous "
    "behaviour; ~0.001 = read the E2 report built 0.8 ms into the step.",
    ns3::DoubleValue (0.0), ns3::MakeDoubleChecker<double> (0.0, 0.05));

// DL offered-load knob for the trafficModel=3 full-buffer UDP UEs (u % 4 == 0).
//
// `ues` used to be the only way to change downlink load, but it conflates two
// things: how many UEs the topology has, which MLB wants high so the CIO lever
// has users to move, and how much DL traffic is offered, which has to sit near
// capacity rather than far above it. Measured at ues=5, 9 full-buffer UEs at
// 500 us offer 188.6 Mbps against about 168 Mbps carried, so the RLC queues ramp
// without bound, hit the 10 MB per-bearer cap and start dropping. No CIO policy
// can fix that network. At ues=3 the queues are stable but a -3 dB offload moves
// nothing (backlog -1.2%, p=0.96). This knob decouples the two, so the load can
// be set near capacity at the UE count that actually has traction.
//
// The 500 us default reproduces the previous hardcoded behaviour exactly.
// bytes/s = 1280 / (interval_us * 1e-6); 500 us is 20.96 Mbps including headers.
static ns3::GlobalValue g_udpFullBufferIntervalUs (
    "udpFullBufferIntervalUs",
    "Inter-packet interval (us) of the full-buffer UDP DL clients in trafficModel=3."
    " 500 = the historical 20.96 Mbps per UE. Larger = less offered DL load.",
    ns3::UintegerValue (500), ns3::MakeUintegerChecker<uint32_t> ());

// DL offered-load knobs for the trafficModel=3 BURSTY UEs (u % 4 == 1, 2, 3).
//
// These used to be UPLINK TCP OnOff applications installed on the UE nodes
// (clientHelperTcp*.Install(ueNodes.Get(u)) with Remote = remoteHost). Every
// reward term and every observation field in MlbZmqEnv measures DOWNLINK --
// DL PRB, DL RLC buffer_bytes, DL MAC volume_bytes, DlE2PdcpStats -- so 26 of
// the 35 UEs at ues=5 were invisible to the agent: measured DL PRB on cells
// carrying no full-buffer UE was 0.0054 over 63,098 cell-steps, and the 26
// bursty UEs delivered 0.71 Mbps of DL between them (TCP ACKs). A downlink
// load-balancing agent cannot balance load that is not on the downlink: CIO
// moved UE COUNT at r = +0.42 but DL PRB at only r = +0.037.
//
// They are now DOWNLINK UDP OnOff flows (sink on the UE, source on the remote
// host). UDP rather than TCP deliberately: TCP would adapt its rate to the
// radio, so a congested cell would back off and partly self-correct the very
// imbalance the agent is meant to fix, and the agent's own action would change
// the offered load. UDP keeps offered load open-loop.
//
// OnTime/OffTime are ExponentialRandomVariable with the default Mean = 1.0, so
// the duty cycle is 50% and the MEAN rate is HALF the DataRate set here.
static ns3::GlobalValue g_burstyDlRate1 (
    "burstyDlRate1", "DataRate of the trafficModel=3 u%4==1 DL bursty UEs"
    " (mean offered load is half this, 50% duty cycle).",
    ns3::StringValue ("5Mbps"), ns3::MakeStringChecker ());
static ns3::GlobalValue g_burstyDlRate2 (
    "burstyDlRate2", "DataRate of the trafficModel=3 u%4==2 DL bursty UEs"
    " (mean offered load is half this, 50% duty cycle).",
    ns3::StringValue ("3Mbps"), ns3::MakeStringChecker ());
static ns3::GlobalValue g_burstyDlRate3 (
    "burstyDlRate3", "DataRate of the trafficModel=3 u%4==3 DL bursty UEs"
    " (mean offered load is half this, 50% duty cycle).",
    ns3::StringValue ("1.5Mbps"), ns3::MakeStringChecker ());

// Mean ON and OFF durations (s) of the trafficModel=3 DL bursty UEs, shared by
// all three bursty classes. Both phases are ExponentialRandomVariable.
//
// At the 1.0 s default a cell's instantaneous DL load reshuffles about once a
// second, faster than a CIO change can usefully follow, so the per-step PRB
// argmax moved away from the episode's hotspot on ~90% of steps (report §10.3).
// Raising these makes an imbalance persist long enough to be worth correcting.
//
// Duty cycle is ON / (ON + OFF), so the mean offered rate is DataRate times
// that; keep ON == OFF to keep burstyDlRate*'s "mean is half" reading. Note
// OnOffApplication starts in OFF: with a mean OFF of T_off, a UE stays silent
// for a whole episode of length L with probability exp(-L / T_off).
//
// The 1.0 defaults reproduce the previous behaviour (Mean=1 is the
// ExponentialRandomVariable default).
static ns3::GlobalValue g_burstyOnMeanS (
    "burstyOnMeanS", "Mean ON duration (s) of the trafficModel=3 DL bursty UEs.",
    ns3::DoubleValue (1.0), ns3::MakeDoubleChecker<double> (0.001));
static ns3::GlobalValue g_burstyOffMeanS (
    "burstyOffMeanS", "Mean OFF duration (s) of the trafficModel=3 DL bursty UEs.",
    ns3::DoubleValue (1.0), ns3::MakeDoubleChecker<double> (0.001));

// Persistent UE hotspot for MLB, only with positionAllocator=0.
//
// positionAllocator=0 drops every UE on one disc of radius isd around the
// network centre, which gives a mild, fixed asymmetry (the centre cell holds
// ~9.6 of 35 UEs, each ring cell ~4.2) and no cell that is clearly overloaded
// next to one with room. These knobs move round(hotspotFraction * N) of the N
// UEs onto a disc of radius hotspotRadius around cell hotspotCellId; the rest
// are placed exactly as before. The total UE count is unchanged, so total
// offered load stays comparable, and the hotspot UEs are taken per traffic class
// (u % 4) in proportion to that class's size, so the hotspot carries the same
// traffic mix as the network.
//
// Positions come from the same UniformDiscPositionAllocator (ns-3 RNG, RngRun
// stream) and mobility is the same RandomWalk2d, so a given RngRun always
// gives the same layout. Unlike positionAllocator=1 there is no wall-clock
// shuffle.
//
// hotspotFraction = 0, the default, reproduces the previous placement exactly.
static ns3::GlobalValue g_hotspotFraction (
    "hotspotFraction",
    "Fraction of UEs placed in a hotspot around hotspotCellId [0, 1]."
    " 0 = off (previous placement). Only with positionAllocator=0.",
    ns3::DoubleValue (0.0), ns3::MakeDoubleChecker<double> (0.0, 1.0));
static ns3::GlobalValue g_hotspotCellId (
    "hotspotCellId", "mmWave cell id the hotspot is centred on (2 = centre cell, 3-8 = ring).",
    ns3::UintegerValue (2), ns3::MakeUintegerChecker<uint16_t> ());
static ns3::GlobalValue g_hotspotRadius (
    "hotspotRadius", "Radius (m) of the hotspot disc around hotspotCellId.",
    ns3::DoubleValue (500.0), ns3::MakeDoubleChecker<double> (1.0));

// Share of the full-buffer UEs (trafficModel=3, u % 4 == 0) placed in the
// hotspot, overriding the proportional quota for that class only.
//
// DL PRB follows the full-buffer UEs, not the UE count: one 1400 us full-buffer
// UE (7.5 Mbps) can hold a 20 MHz cell near 0.7 PRB on its own, so a hotspot
// with a proportional 4 of 9 full-buffer UEs was not reliably the PRB hotspot.
// With a share s, round(s * nFullBuffer) of them go to the hotspot (0.78 -> 7
// of 9 at 35 UEs). The other classes keep their hotspot quota and the total UE
// count is unchanged, so the hotspot grows by the extra full-buffer UEs.
//
// -1, the default, keeps the proportional quota (the hotspotFraction-only
// behaviour). Requires hotspotFraction > 0.
// Keep hotspot UEs inside the r = isd disc the other UEs are placed on.
//
// A ring-cell hotspot disc of radius 500 m lies 55% outside that disc, on the
// side with no neighbour cell, where no CIO can offload anyone (audit_oct_2.md
// L9). When true, each hotspot UE's position is redrawn from the same hotspot
// allocator (same RNG stream) until it is within isd of the network centre.
// No effect for hotspotCellId = 2, whose disc is already inside. Requires
// hotspotFraction > 0. false, the default, reproduces the previous placement.
static ns3::GlobalValue g_hotspotClipToUeDisc (
    "hotspotClipToUeDisc",
    "Redraw hotspot UE positions until they lie within isd of the network centre.",
    ns3::BooleanValue (false), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_hotspotFbShare (
    "hotspotFbShare",
    "Share [0, 1] of the full-buffer UEs (u%4==0) placed in the hotspot."
    " -1 = proportional to hotspotFraction (default).",
    ns3::DoubleValue (-1.0), ns3::MakeDoubleChecker<double> (-1.0, 1.0));

static ns3::GlobalValue g_trafficModel (
    "trafficModel",
    "Type of the traffic model at the transport layer [0,3],"
    " can generate full buffer traffic (0),"
    " half nodes in full buffer and half nodes in bursty (1),"
    " bursty traffic (2),"
    " Mixed (3): 0.25 full buffer, 0.25 bursty 3Mbps, 0.25 bursty 0.75Mbps, 0.25 bursty 0.15Mbps",
    ns3::UintegerValue (0), ns3::MakeUintegerChecker<uint8_t> ());

static ns3::GlobalValue g_nBsNoUesAlloc (
    "nBsNoUesAlloc",
    "Number of BS without UEs allocated [0, 1, 2, 3], "
    "-1 is default value: no BSs are choosen",
    ns3::IntegerValue (-1), ns3::MakeIntegerChecker<int8_t> ());

static ns3::GlobalValue g_positionAllocator (
    "positionAllocator",
    "Type of the positionAllocator of UEs [0,1],"
    " Uniform random distribution of UEs on discs around each BS  (0),"
    " Uniform random distribution of UEs on discs around nBS-nBsNoUesAlloc (1)",
    ns3::UintegerValue (0), ns3::MakeUintegerChecker<uint8_t> ());

static ns3::GlobalValue g_configuration ("configuration",
                                         "Set the wanted configuration to emulate [0,2]",
                                         ns3::UintegerValue (1),
                                         ns3::MakeUintegerChecker<uint8_t> ());
static ns3::GlobalValue
    g_dataRate ("dataRate", "Set the data rate to be used [only \"0\"(low),\"1\"(high) admitted]",
                ns3::DoubleValue (0), ns3::MakeDoubleChecker<double> (0, 1));

static ns3::GlobalValue g_ues ("ues", "Number of UEs for each mmWave ENB.", ns3::UintegerValue (7),
                               ns3::MakeUintegerChecker<uint8_t> ());

static ns3::GlobalValue g_indicationPeriodicity ("indicationPeriodicity", "E2 Indication Periodicity reports (value in seconds)", ns3::DoubleValue (0.1),
                                   ns3::MakeDoubleChecker<double> (0.01, 2.0));

static ns3::GlobalValue g_simTime ("simTime", "Simulation time in seconds", ns3::DoubleValue (1.9),
                                   ns3::MakeDoubleChecker<double> (0.1, 1000.0));

static ns3::GlobalValue g_reducedPmValues ("reducedPmValues", "If true, use a subset of the the pm containers",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_outageThreshold ("outageThreshold",
                                           "SNR threshold for outage events [dB]",
                                           ns3::DoubleValue (-1000.0),
                                           ns3::MakeDoubleChecker<double> ());

static ns3::GlobalValue g_basicCellId ("basicCellId", "The next value will be the first cellId",
                                       ns3::UintegerValue (1),
                                       ns3::MakeUintegerChecker<uint8_t> ());

static ns3::GlobalValue g_numberOfRaPreambles ("numberOfRaPreambles", "how many random access preambles are available for the contention based RACH process",
                                       ns3::UintegerValue (30), // TS use case should be 40, 52 is default, ES should be 30
                                       ns3::MakeUintegerChecker<uint8_t> ());

static ns3::GlobalValue
    g_handoverMode ("handoverMode",
                    "HO euristic to be used,"
                    "can be only \"NoAuto\", \"FixedTtt\", \"DynamicTtt\",   \"Threshold\"",
                    ns3::StringValue ("NoAuto"), ns3::MakeStringChecker ());

static ns3::GlobalValue g_e2TermIp ("e2TermIp", "The IP address of the RIC E2 termination",
                                    ns3::StringValue ("10.244.0.240"), ns3::MakeStringChecker ());

static ns3::GlobalValue
    g_enableE2FileLogging ("enableE2FileLogging",
              "If true, generate offline file logging instead of connecting to RIC",
              ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_controlFileName ("controlFileName", "The path to the control file (can be absolute)",
                                     ns3::StringValue (""), ns3::MakeStringChecker ());

static ns3::GlobalValue q_useSemaphores ("useSemaphores", "If true, enables the use of semaphores for external environment control",
                                        ns3::BooleanValue (false), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_minSpeed ("minSpeed",
                                           "minimum UE speed in m/s",
                                           ns3::DoubleValue (2.0),
                                           ns3::MakeDoubleChecker<double> ());

static ns3::GlobalValue g_maxSpeed ("maxSpeed",
                                           "maximum UE speed in m/s",
                                           ns3::DoubleValue (4.0),
                                           ns3::MakeDoubleChecker<double> ());

static ns3::GlobalValue g_heuristic (
    "heuristicType",
    "Type of heuristic for managing BS status,"
    " No heuristic (-1),"
    " Random sleeping (0),"
    " Static sleeping (1),"
    " Dynamic sleeping (2),",
    ns3::IntegerValue (-1), ns3::MakeIntegerChecker<int8_t> ());
static ns3::GlobalValue g_probOn (
    "probOn",
    "Probability to turn BS ON for the random sleeping heuristic"
    "the value is proposed on the paper 'Small Cell Base Station Sleep"
    "Strategies for Energy Efficiency' in order to obtain an overall "
    "small average cell wake up time"
    "https://ieeexplore.ieee.org/abstract/document/7060678",
    ns3::DoubleValue (0.6038), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_probIdle (
    "probIdle",
    "Probability to turn BS Idle for the random sleeping heuristic"
    "the value is proposed on the paper 'Small Cell Base Station Sleep"
    "Strategies for 'Energy Efficiency' in order to obtain an overall" 
    "small average cell wake up time"
    "https://ieeexplore.ieee.org/abstract/document/7060678",
    ns3::DoubleValue (0.3854), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_probSleep (
    "probSleep",
    "Probability to turn BS Sleep for the random sleeping heuristic"
    "the value is proposed on the paper 'Small Cell Base Station Sleep"
    "Strategies for Energy Efficiency' in order to obtain an overall" 
    "small average cell wake up time"
    "https://ieeexplore.ieee.org/abstract/document/7060678",
    ns3::DoubleValue (0.0107), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_probOff (
    "probOff",
    "Probability to turn BS Off for the random sleeping heuristic"
    "the value is proposed on the paper 'Small Cell Base Station Sleep"
    "Strategies for Energy Efficiency' in order to obtain an overall" 
    "small average cell wake up time"
    "https://ieeexplore.ieee.org/abstract/document/7060678",
    ns3::DoubleValue (0.0), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_sinrTh (
    "sinrTh",
    "SINR threshold for static and dynamic sleeping heuristic",
    ns3::DoubleValue (73.0), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_bsOn (
    "bsOn",
    "number of BS to turn ON for static and dynamic sleeping heuristic",
    ns3::UintegerValue (2), ns3::MakeUintegerChecker<uint8_t> ());
static ns3::GlobalValue g_bsIdle (
    "bsIdle",
    "number of BS to turn IDLE for static and dynamic sleeping heuristic",
    ns3::UintegerValue (2), ns3::MakeUintegerChecker<uint8_t> ());
static ns3::GlobalValue g_bsSleep (
    "bsSleep",
    "number of BS to turn Sleep for static and dynamic sleeping heuristic",
    ns3::UintegerValue (2), ns3::MakeUintegerChecker<uint8_t> ());
static ns3::GlobalValue g_bsOff (
    "bsOff",
    "number of BS to turn Off for static and dynamic sleeping heuristic",
    ns3::UintegerValue (1), ns3::MakeUintegerChecker<uint8_t> ());

int
main (int argc, char *argv[])
{
  LogComponentEnableAll (LOG_PREFIX_ALL);
  // LogComponentEnable ("ScenarioThree", LOG_LEVEL_DEBUG);
  // LogComponentEnable ("EnergyHeuristic", LOG_LEVEL_DEBUG);
  // LogComponentEnable ("PacketSink", LOG_LEVEL_ALL);
  // LogComponentEnable ("OnOffApplication", LOG_LEVEL_ALL);
  // LogComponentEnable ("LtePdcp", LOG_LEVEL_ALL);
  // LogComponentEnable ("LteRlcAm", LOG_LEVEL_ALL);
  // LogComponentEnable ("MmWaveUeMac", LOG_LEVEL_ALL);
  // LogComponentEnable ("MmWaveEnbMac", LOG_LEVEL_ALL);
  // LogComponentEnable ("LteUeMac", LOG_LEVEL_ALL);
  // LogComponentEnable ("LteEnbMac", LOG_LEVEL_ALL);
  // LogComponentEnable ("MmWaveFlexTtiMacScheduler", LOG_LEVEL_ALL);
  // LogComponentEnable ("LteEnbRrc", LOG_LEVEL_ALL);
  // LogComponentEnable ("LteUeRrc", LOG_LEVEL_ALL);
  // LogComponentEnable ("McEnbPdcp", LOG_LEVEL_ALL);
  // LogComponentEnable ("McUePdcp", LOG_LEVEL_ALL);
  // LogComponentEnable ("ScenarioThree``", LOG_LEVEL_ALL);
  // LogComponentEnable ("RicControlMessage", LOG_LEVEL_ALL);
  // LogComponentEnable ("Asn1Types", LOG_LEVEL_LOGIC);
  // LogComponentEnable ("E2Termination", LOG_LEVEL_LOGIC);
  // LogComponentEnable ("MmWaveSpectrumPhy", LOG_LEVEL_ALL);

  // The maximum X coordinate of the scenario
  double maxXAxis = 4000;
  // The maximum Y coordinate of the scenario
  double maxYAxis = 4000;

  // Command line arguments
  CommandLine cmd;
  cmd.Parse (argc, argv);

  bool harqEnabled = true;

  UintegerValue uintegerValue;
  IntegerValue integerValue;
  BooleanValue booleanValue;
  StringValue stringValue;
  DoubleValue doubleValue;

  GlobalValue::GetValueByName ("dataRate", doubleValue);
  double dataRateFromConf = doubleValue.Get ();
  GlobalValue::GetValueByName ("rlcAmEnabled", booleanValue);
  bool rlcAmEnabled = booleanValue.Get ();
  GlobalValue::GetValueByName ("bufferSize", uintegerValue);
  uint32_t bufferSize = uintegerValue.Get ();
  GlobalValue::GetValueByName ("basicCellId", uintegerValue);
  uint16_t basicCellId = uintegerValue.Get ();
  GlobalValue::GetValueByName ("enableTraces", booleanValue);
  bool enableTraces = booleanValue.Get ();
  GlobalValue::GetValueByName ("trafficModel", uintegerValue);
  uint8_t trafficModel = uintegerValue.Get ();
  GlobalValue::GetValueByName ("udpFullBufferIntervalUs", uintegerValue);
  uint32_t udpFullBufferIntervalUs = uintegerValue.Get ();
  GlobalValue::GetValueByName ("zmqPort", uintegerValue);
  uint16_t zmqPort = uintegerValue.Get ();
  StringValue stringValueTmp;
  GlobalValue::GetValueByName ("burstyDlRate1", stringValueTmp);
  std::string burstyDlRate1 = stringValueTmp.Get ();
  GlobalValue::GetValueByName ("burstyDlRate2", stringValueTmp);
  std::string burstyDlRate2 = stringValueTmp.Get ();
  GlobalValue::GetValueByName ("burstyDlRate3", stringValueTmp);
  std::string burstyDlRate3 = stringValueTmp.Get ();
  DoubleValue doubleValueTmp;
  GlobalValue::GetValueByName ("burstyOnMeanS", doubleValueTmp);
  double burstyOnMeanS = doubleValueTmp.Get ();
  GlobalValue::GetValueByName ("burstyOffMeanS", doubleValueTmp);
  double burstyOffMeanS = doubleValueTmp.Get ();
  GlobalValue::GetValueByName ("hotspotFraction", doubleValueTmp);
  double hotspotFraction = doubleValueTmp.Get ();
  GlobalValue::GetValueByName ("hotspotRadius", doubleValueTmp);
  double hotspotRadius = doubleValueTmp.Get ();
  GlobalValue::GetValueByName ("hotspotClipToUeDisc", booleanValue);
  bool hotspotClipToUeDisc = booleanValue.Get ();
  GlobalValue::GetValueByName ("hotspotFbShare", doubleValueTmp);
  double hotspotFbShare = doubleValueTmp.Get ();
  GlobalValue::GetValueByName ("hotspotCellId", uintegerValue);
  uint16_t hotspotCellId = uintegerValue.Get ();
  GlobalValue::GetValueByName ("controlInterval", doubleValueTmp);
  double controlIntervalCfg = doubleValueTmp.Get ();
  GlobalValue::GetValueByName ("handoverHysteresisDb", doubleValueTmp);
  double handoverHysteresisDb = doubleValueTmp.Get ();
  if (handoverHysteresisDb > 0.0)
    {
      Config::SetDefault ("ns3::LteEnbRrc::HandoverHysteresisDb",
                          DoubleValue (handoverHysteresisDb));
    }
  GlobalValue::GetValueByName ("channelConditionUpdatePeriodMs", uintegerValue);
  uint32_t channelConditionUpdatePeriodMs = uintegerValue.Get ();
  if (channelConditionUpdatePeriodMs != 100)
    {
      NS_LOG_UNCOND ("channelConditionUpdatePeriodMs " << channelConditionUpdatePeriodMs);
    }
  GlobalValue::GetValueByName ("logChannelConditions", booleanValue);
  bool logChannelConditions = booleanValue.Get ();
  GlobalValue::GetValueByName ("handoverSinrFilterTauMs", doubleValueTmp);
  double handoverSinrFilterTauMs = doubleValueTmp.Get ();
  if (handoverSinrFilterTauMs > 0.0)
    {
      Config::SetDefault ("ns3::LteEnbRrc::HandoverSinrFilterTauMs",
                          DoubleValue (handoverSinrFilterTauMs));
      g_marlEmitHoDecisionSinr = true;
      NS_LOG_UNCOND ("handoverSinrFilterTauMs " << handoverSinrFilterTauMs);
    }
  BooleanValue tttFromBiasedSinrValue;
  GlobalValue::GetValueByName ("tttFromBiasedSinr", tttFromBiasedSinrValue);
  if (tttFromBiasedSinrValue.Get ())
    {
      Config::SetDefault ("ns3::LteEnbRrc::TttFromBiasedSinr", BooleanValue (true));
    }
  if (handoverHysteresisDb > 0.0 || tttFromBiasedSinrValue.Get ())
    {
      NS_LOG_UNCOND ("handoverHysteresisDb " << handoverHysteresisDb << " tttFromBiasedSinr "
                     << tttFromBiasedSinrValue.Get ());
    }
  GlobalValue::GetValueByName ("controlPhaseOffsetS", doubleValueTmp);
  double controlPhaseOffsetS = doubleValueTmp.Get ();
  if (controlPhaseOffsetS > 0.0)
    {
      // Before any device exists, so every MmWaveEnbNetDevice picks it up.
      Config::SetDefault ("ns3::MmWaveEnbNetDevice::MarlBankSinrBins", BooleanValue (true));
      g_marlUseBankedSinrBins = true;
    }
  GlobalValue::GetValueByName ("nBsNoUesAlloc", integerValue);
  int8_t nBsNoUesAlloc = integerValue.Get ();
  GlobalValue::GetValueByName ("positionAllocator", uintegerValue);
  uint8_t positionAllocator = uintegerValue.Get ();
  GlobalValue::GetValueByName ("outageThreshold",doubleValue);
  double outageThreshold = doubleValue.Get ();
  GlobalValue::GetValueByName ("handoverMode", stringValue);
  std::string handoverMode = stringValue.Get ();
  GlobalValue::GetValueByName ("e2TermIp", stringValue);
  std::string e2TermIp = stringValue.Get ();
  GlobalValue::GetValueByName ("enableE2FileLogging", booleanValue);
  bool enableE2FileLogging = booleanValue.Get ();
  GlobalValue::GetValueByName ("minSpeed", doubleValue);
  double minSpeed = doubleValue.Get ();
  GlobalValue::GetValueByName ("maxSpeed", doubleValue);
  double maxSpeed = doubleValue.Get ();
  GlobalValue::GetValueByName ("numberOfRaPreambles", uintegerValue);
  uint8_t numberOfRaPreambles = uintegerValue.Get ();

  // Heuristic parameters
  GlobalValue::GetValueByName ("heuristicType", integerValue);
  int8_t heuristicType = integerValue.Get ();
  GlobalValue::GetValueByName ("probOn", doubleValue);
  double probOn = doubleValue.Get ();
  GlobalValue::GetValueByName ("probIdle", doubleValue);
  double probIdle = doubleValue.Get ();
  GlobalValue::GetValueByName ("probSleep", doubleValue);
  double probSleep = doubleValue.Get ();
  GlobalValue::GetValueByName ("probOff", doubleValue);
  double probOff = doubleValue.Get ();
  GlobalValue::GetValueByName ("sinrTh", doubleValue);
  double sinrTh = doubleValue.Get ();
  GlobalValue::GetValueByName ("bsOn", uintegerValue);
  int bsOn = uintegerValue.Get ();
  GlobalValue::GetValueByName ("bsIdle", uintegerValue);
  int bsIdle = uintegerValue.Get ();
  GlobalValue::GetValueByName ("bsSleep", uintegerValue);
  int bsSleep = uintegerValue.Get ();
  GlobalValue::GetValueByName ("bsOff", uintegerValue);
  int bsOff = uintegerValue.Get ();


  NS_LOG_UNCOND ("rlcAmEnabled " << rlcAmEnabled << " bufferSize " << bufferSize
                                 << " traffic Model " << unsigned (trafficModel)
                                 << " OutageThreshold " << outageThreshold << " HandoverMode "
                                 << handoverMode << " BasicCellId " << basicCellId << " e2TermIp "
                                 << e2TermIp << " enableE2FileLogging " << enableE2FileLogging
                                 << " minSpeed " << minSpeed << " maxSpeed " << maxSpeed);

  GlobalValue::GetValueByName ("e2lteEnabled", booleanValue);
  bool e2lteEnabled = booleanValue.Get ();
  GlobalValue::GetValueByName ("e2nrEnabled", booleanValue);
  bool e2nrEnabled = booleanValue.Get ();
  GlobalValue::GetValueByName ("e2du", booleanValue);
  bool e2du = booleanValue.Get ();
  if (controlPhaseOffsetS > 0.0 && !e2du)
    {
      // MarlControlStep then reads the banked SINR bins, which only the DU
      // report (EnableDuReport) writes: sinr_bins would stay zero all run.
      NS_FATAL_ERROR ("controlPhaseOffsetS > 0 needs e2du=true: the banked SINR bins "
                      "are filled by the DU report only");
    }
  GlobalValue::GetValueByName ("e2cuUp", booleanValue);
  bool e2cuUp = booleanValue.Get ();
  GlobalValue::GetValueByName ("e2cuCp", booleanValue);
  bool e2cuCp = booleanValue.Get ();

  GlobalValue::GetValueByName ("reducedPmValues", booleanValue);
  bool reducedPmValues = booleanValue.Get ();

  GlobalValue::GetValueByName ("indicationPeriodicity", doubleValue);
  double indicationPeriodicity = doubleValue.Get ();

  GlobalValue::GetValueByName ("controlFileName", stringValue);
  std::string controlFilename = stringValue.Get ();

  GlobalValue::GetValueByName ("useSemaphores", booleanValue);
  bool useSemaphores = booleanValue.Get ();

    NS_LOG_UNCOND("e2lteEnabled " << e2lteEnabled 
    << " e2nrEnabled " << e2nrEnabled
    << " e2du " << e2du
    << " e2cuCp " << e2cuCp
    << " e2cuUp " << e2cuUp
    << " reducedPmValues " << reducedPmValues 
    << " controlFilename " << controlFilename
    << " useSemaphores " << useSemaphores
    << " indicationPeriodicity " << indicationPeriodicity
    << " heuristicType " << int(heuristicType)
  );

  Config::SetDefault ("ns3::LteEnbNetDevice::UseSemaphores", BooleanValue (useSemaphores));
  Config::SetDefault ("ns3::LteEnbNetDevice::ControlFileName", StringValue(controlFilename));
  Config::SetDefault ("ns3::LteEnbNetDevice::E2Periodicity", DoubleValue (indicationPeriodicity));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::E2Periodicity", DoubleValue (indicationPeriodicity));

  Config::SetDefault ("ns3::MmWaveHelper::E2Periodicity", DoubleValue (indicationPeriodicity));
  Config::SetDefault ("ns3::MmWaveHelper::E2ModeLte", BooleanValue(e2lteEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::E2ModeNr", BooleanValue(e2nrEnabled));
  
  // The DU PM reports should come from both NR gNB as well as LTE eNB, 
  // since in the RLC/MAC/PHY entities are present in BOTH NR gNB as well as LTE eNB.
  // TODO DU reports from LTE eNB are not implemented yet
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableDuReport", BooleanValue(e2du));

  // The CU-UP PM reports should only come from LTE eNB, since in the NS3 “EN-DC 
  // simulation (Option 3A)”, the PDCP is only in the LTE eNB and NOT in the NR gNB
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableCuUpReport", BooleanValue(e2cuUp));
  Config::SetDefault ("ns3::LteEnbNetDevice::EnableCuUpReport", BooleanValue(e2cuUp));

  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableCuCpReport", BooleanValue(e2cuCp));
  Config::SetDefault ("ns3::LteEnbNetDevice::EnableCuCpReport", BooleanValue(e2cuCp));
  
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::ReducedPmValues", BooleanValue (reducedPmValues));
  Config::SetDefault ("ns3::LteEnbNetDevice::ReducedPmValues", BooleanValue (reducedPmValues));

  Config::SetDefault ("ns3::LteEnbNetDevice::EnableE2FileLogging", BooleanValue (enableE2FileLogging));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableE2FileLogging", BooleanValue (enableE2FileLogging));

  Config::SetDefault ("ns3::MmWaveEnbMac::NumberOfRaPreambles", UintegerValue (numberOfRaPreambles));

  Config::SetDefault ("ns3::MmWaveHelper::RlcAmEnabled", BooleanValue (rlcAmEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::UseIdealRrc", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveHelper::BasicCellId", UintegerValue (basicCellId));
  Config::SetDefault ("ns3::MmWaveHelper::BasicImsi", UintegerValue ((basicCellId-1)));
  Config::SetDefault ("ns3::MmWaveHelper::E2TermIp", StringValue (e2TermIp));

  Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::NumHarqProcess", UintegerValue (100));
  //Config::SetDefault ("ns3::MmWaveBearerStatsCalculator::EpochDuration", TimeValue (MilliSeconds (10.0)));

  Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (100.0)));
  Config::SetDefault ("ns3::ThreeGppChannelConditionModel::UpdatePeriod", TimeValue (MilliSeconds (channelConditionUpdatePeriodMs)));

  Config::SetDefault ("ns3::LteRlcAm::ReportBufferStatusTimer", TimeValue (MilliSeconds (10.0)));
  Config::SetDefault ("ns3::LteRlcUmLowLat::ReportBufferStatusTimer",
                      TimeValue (MilliSeconds (10.0)));
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (bufferSize * 1024 * 1024));
  Config::SetDefault ("ns3::LteRlcUmLowLat::MaxTxBufferSize",
                      UintegerValue (bufferSize * 1024 * 1024));
  Config::SetDefault ("ns3::LteRlcAm::MaxTxBufferSize", UintegerValue (bufferSize * 1024 * 1024));

  Config::SetDefault ("ns3::LteEnbRrc::OutageThreshold", DoubleValue (outageThreshold));
  Config::SetDefault ("ns3::LteEnbRrc::SecondaryCellHandoverMode", StringValue (handoverMode));

  // Carrier bandwidth in Hz
  double bandwidth;
  // Center frequency in Hz
  double centerFrequency;
  // Distance between the mmWave BSs and the two co-located LTE and mmWave BSs in meters
  double isd; // (interside distance)
  // Number of antennas in each UE
  int numAntennasMcUe;
  // Number of antennas in each mmWave BS
  int numAntennasMmWave;
  // Data rate of transport layer
  std::string dataRate;

  GlobalValue::GetValueByName ("configuration", uintegerValue);
  uint8_t configuration = uintegerValue.Get ();
  switch (configuration)
    {
    case 0:
      centerFrequency = 850e6;
      bandwidth = 20e6;
      isd = 1000;
      numAntennasMcUe = 1;
      numAntennasMmWave = 1;
      dataRate = (dataRateFromConf == 0 ? "1.5Mbps" : "4.5Mbps");
      break;

    case 1:
      centerFrequency = 3.5e9;
      bandwidth = 20e6;
      isd = 1000;
      numAntennasMcUe = 1;
      numAntennasMmWave = 1;
      dataRate = (dataRateFromConf == 0 ? "1.5Mbps" : "4.5Mbps");
      break;

    case 2:
      centerFrequency = 28e9;
      bandwidth = 100e6;
      isd = 200;
      numAntennasMcUe = 16;
      numAntennasMmWave = 64;
      dataRate = (dataRateFromConf == 0 ? "15Mbps" : "45Mbps");
      break;

    default:
      NS_FATAL_ERROR ("Configuration not recognized" << configuration);
      break;
    }

  NS_LOG_INFO ("Bandwidth " << bandwidth << " centerFrequency " << double (centerFrequency)
                            << " isd " << isd << " numAntennasMcUe " << numAntennasMcUe
                            << " numAntennasMmWave " << numAntennasMmWave << " dataRate "
                            << dataRate);

  Config::SetDefault ("ns3::MmWavePhyMacCommon::Bandwidth", DoubleValue (bandwidth));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue (centerFrequency));

  Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper> ();
  mmwaveHelper->SetPathlossModelType ("ns3::ThreeGppUmiStreetCanyonPropagationLossModel");
  mmwaveHelper->SetChannelConditionModelType ("ns3::ThreeGppUmiStreetCanyonChannelConditionModel");

  // Set the number of antennas in the devices
  mmwaveHelper->SetUePhasedArrayModelAttribute("NumColumns", UintegerValue(std::sqrt(numAntennasMcUe)));
  mmwaveHelper->SetUePhasedArrayModelAttribute("NumRows", UintegerValue(std::sqrt(numAntennasMcUe)));
  mmwaveHelper->SetEnbPhasedArrayModelAttribute("NumColumns",UintegerValue(std::sqrt(numAntennasMmWave)));
  mmwaveHelper->SetEnbPhasedArrayModelAttribute("NumRows", UintegerValue(std::sqrt(numAntennasMmWave)));

  Ptr<MmWavePointToPointEpcHelper> epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
  mmwaveHelper->SetEpcHelper (epcHelper);

  uint8_t nMmWaveEnbNodes = 7;
  uint8_t nLteEnbNodes = 1;
  GlobalValue::GetValueByName ("ues", uintegerValue);
  uint32_t ues = uintegerValue.Get ();
  uint8_t nUeNodes = ues * nMmWaveEnbNodes;

  NS_LOG_INFO (" Bandwidth " << bandwidth << " centerFrequency " << double (centerFrequency)
                             << " isd " << isd << " numAntennasMcUe " << numAntennasMcUe
                             << " numAntennasMmWave " << numAntennasMmWave << " dataRate "
                             << dataRate << " nMmWaveEnbNodes " << unsigned (nMmWaveEnbNodes));

  // Get SGW/PGW and create a single RemoteHost
  Ptr<Node> pgw = epcHelper->GetPgwNode ();
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  // Create the Internet by connecting remoteHost to pgw. Setup routing too
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (2500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  // interface 0 is localhost, 1 is the p2p device
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  // create LTE, mmWave eNB nodes and UE node
  NodeContainer ueNodes;
  NodeContainer mmWaveEnbNodes;
  NodeContainer lteEnbNodes;
  NodeContainer allEnbNodes;
  mmWaveEnbNodes.Create (nMmWaveEnbNodes);
  lteEnbNodes.Create (nLteEnbNodes);
  ueNodes.Create (nUeNodes);
  allEnbNodes.Add (lteEnbNodes);
  allEnbNodes.Add (mmWaveEnbNodes);

  // Position
  Vector centerPosition = Vector (maxXAxis / 2, maxYAxis / 2, 3);

  // Install Mobility Model
  Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();

  // We want a center with one LTE enb and one mmWave co-located in the same place
  enbPositionAlloc->Add (centerPosition);
  enbPositionAlloc->Add (centerPosition);

  double x;
  double y;
  double nConstellation = nMmWaveEnbNodes - 1;

  // This guarantee that each of the rest BSs is placed at the same distance from the two co-located in the center
  for (int8_t i = 0; i < nConstellation; ++i)
    {
      x = isd * cos ((2 * M_PI * i) / (nConstellation));
      y = isd * sin ((2 * M_PI * i) / (nConstellation));
      enbPositionAlloc->Add (Vector (centerPosition.x + x, centerPosition.y + y, 3));
    }

  MobilityHelper enbmobility;
  enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  enbmobility.SetPositionAllocator (enbPositionAlloc);
  enbmobility.Install (allEnbNodes);

  Ptr<UniformDiscPositionAllocator> uePositionAlloc = CreateObject<UniformDiscPositionAllocator> ();

  Ptr<UniformRandomVariable> speed = CreateObject<UniformRandomVariable> ();
  speed->SetAttribute ("Min", DoubleValue (minSpeed));
  speed->SetAttribute ("Max", DoubleValue (maxSpeed));
  Ptr<UniformRandomVariable> puntTimeDirection = CreateObject<UniformRandomVariable> ();
  // Set min and max speed of UEs
  puntTimeDirection->SetAttribute ("Min", DoubleValue (1));
  puntTimeDirection->SetAttribute ("Max", DoubleValue (3));
  double timeDirection=puntTimeDirection->GetValue();

  // Position allocator:
  //  0: UEs set over a UniformDiscPositionAllocator with radius = isd and RandomWalk2dMobilityModel 
  //  1: UEs set over a UniformDiscPositionAllocator placed on each BS-nBsNoUesAlloc with radius = isd/2 and RandomWalk2dMobilityModel
  MobilityHelper uemobility;
  switch (positionAllocator)
  {
  case 0: {
      if (nBsNoUesAlloc != -1)
      {
          NS_FATAL_ERROR("nBsNoUesAlloc not correct for selected positionAllocator " << nBsNoUesAlloc << positionAllocator);
      }
      uePositionAlloc->SetX(centerPosition.x);
      uePositionAlloc->SetY(centerPosition.y);
      uePositionAlloc->SetRho(isd);

      uemobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                                  "Mode",
                                  StringValue("Time"),
                                  "Time",
                                  StringValue(std::to_string(timeDirection) + "s"),
                                  "Speed",
                                  PointerValue(speed),
                                  "Bounds",
                                  RectangleValue(Rectangle(0, maxXAxis, 0, maxYAxis)));
      uemobility.SetPositionAllocator(uePositionAlloc);
      if (hotspotClipToUeDisc && hotspotFraction <= 0.0)
      {
          NS_FATAL_ERROR("hotspotClipToUeDisc needs hotspotFraction > 0");
      }
      if (hotspotFbShare >= 0.0 && hotspotFraction <= 0.0)
      {
          NS_FATAL_ERROR("hotspotFbShare needs hotspotFraction > 0");
      }
      if (hotspotFraction <= 0.0)
      {
          uemobility.Install(ueNodes);
          break;
      }

      // Hotspot (see g_hotspotFraction). Pick how many UEs of each traffic class
      // u % 4 go to the hotspot: floor(f * classSize), then the leftover seats of
      // round(f * N) go to the classes with the largest fractional parts, ties to
      // the lower class. Within a class the lowest indices are taken, so the set
      // depends only on (f, N), never on the RNG.
      if (hotspotCellId < 2 || hotspotCellId > 1 + nMmWaveEnbNodes)
      {
          NS_FATAL_ERROR("hotspotCellId " << hotspotCellId << " is not a mmWave cell [2, "
                         << 1 + nMmWaveEnbNodes << "]");
      }
      Vector hotspotCenter =
          mmWaveEnbNodes.Get(hotspotCellId - 2)->GetObject<MobilityModel>()->GetPosition();
      if (hotspotCenter.x - hotspotRadius < 0 || hotspotCenter.x + hotspotRadius > maxXAxis ||
          hotspotCenter.y - hotspotRadius < 0 || hotspotCenter.y + hotspotRadius > maxYAxis)
      {
          NS_FATAL_ERROR("hotspot disc of radius " << hotspotRadius << " around "
                         << hotspotCenter << " leaves the mobility bounds");
      }
      uint32_t nHotspot = static_cast<uint32_t>(std::lround(hotspotFraction * nUeNodes));
      uint32_t classSize[4] = {0, 0, 0, 0};
      for (uint32_t u = 0; u < nUeNodes; ++u)
      {
          classSize[u % 4]++;
      }
      uint32_t classQuota[4];
      double classRemainder[4];
      uint32_t assigned = 0;
      for (int c = 0; c < 4; ++c)
      {
          double exact = hotspotFraction * classSize[c];
          classQuota[c] = static_cast<uint32_t>(std::floor(exact));
          classRemainder[c] = exact - classQuota[c];
          assigned += classQuota[c];
      }
      while (assigned < nHotspot)
      {
          int best = -1;
          for (int c = 0; c < 4; ++c)
          {
              if (classQuota[c] < classSize[c] &&
                  (best < 0 || classRemainder[c] > classRemainder[best]))
              {
                  best = c;
              }
          }
          classQuota[best]++;
          classRemainder[best] = -1.0;
          assigned++;
      }
      if (hotspotFbShare >= 0.0)
      {
          classQuota[0] =
              static_cast<uint32_t>(std::lround(hotspotFbShare * classSize[0]));
      }

      NodeContainer hotspotUes;
      NodeContainer otherUes;
      std::vector<bool> inHotspot(nUeNodes, false);
      uint32_t taken[4] = {0, 0, 0, 0};
      for (uint32_t u = 0; u < nUeNodes; ++u)
      {
          if (taken[u % 4] < classQuota[u % 4])
          {
              taken[u % 4]++;
              inHotspot[u] = true;
              hotspotUes.Add(ueNodes.Get(u));
          }
          else
          {
              otherUes.Add(ueNodes.Get(u));
          }
      }

      // Same allocator object, re-centred, so no new RNG stream is created.
      uePositionAlloc->SetX(hotspotCenter.x);
      uePositionAlloc->SetY(hotspotCenter.y);
      uePositionAlloc->SetRho(hotspotRadius);
      if (hotspotClipToUeDisc)
      {
          Ptr<ListPositionAllocator> clipped = CreateObject<ListPositionAllocator>();
          for (uint32_t i = 0; i < hotspotUes.GetN(); ++i)
          {
              Vector p = uePositionAlloc->GetNext();
              while (CalculateDistance(p, centerPosition) > isd)
              {
                  p = uePositionAlloc->GetNext();
              }
              clipped->Add(p);
          }
          uemobility.SetPositionAllocator(clipped);
          uemobility.Install(hotspotUes);
          uemobility.SetPositionAllocator(uePositionAlloc);
      }
      else
      {
          uemobility.Install(hotspotUes);
      }
      uePositionAlloc->SetX(centerPosition.x);
      uePositionAlloc->SetY(centerPosition.y);
      uePositionAlloc->SetRho(isd);
      uemobility.Install(otherUes);

      NS_LOG_UNCOND("Hotspot: " << hotspotUes.GetN() << " of " << unsigned(nUeNodes)
                    << " UEs on a " << hotspotRadius << " m disc around cell "
                    << hotspotCellId << " at " << hotspotCenter << ", per class u%4 = "
                    << classQuota[0] << "/" << classQuota[1] << "/" << classQuota[2] << "/"
                    << classQuota[3] << " of " << classSize[0] << "/" << classSize[1] << "/"
                    << classSize[2] << "/" << classSize[3]);

      // Layout record for checking: index, class, hotspot flag, start position.
      std::ofstream layout("hotspot_layout.txt", std::ios_base::out | std::ios_base::trunc);
      layout << "u class in_hotspot x y z" << std::endl;
      for (uint32_t u = 0; u < nUeNodes; ++u)
      {
          Vector p = ueNodes.Get(u)->GetObject<MobilityModel>()->GetPosition();
          layout << u << " " << u % 4 << " " << inHotspot[u] << " " << p.x << " " << p.y
                 << " " << p.z << std::endl;
      }
      break;
  }

  case 1: {
      if (hotspotFraction > 0.0)
      {
          NS_FATAL_ERROR("hotspotFraction is only implemented for positionAllocator=0");
      }
      if (nBsNoUesAlloc == -1)
      {
          NS_FATAL_ERROR("nBsNoUesAlloc not correct for selected positionAllocator " << nBsNoUesAlloc << positionAllocator);
      }
      Vector bsCoords[7] = {};
      // save vector coords into an array to shuffle them (without the first LTE BS coordinates)
      enbPositionAlloc->GetNext();
      for (int i = 0; i < nMmWaveEnbNodes; i++)
      {
        bsCoords[i] = enbPositionAlloc->GetNext();
      }
      std::srand(std::time(0));
      std::random_shuffle(std::begin(bsCoords), std::end(bsCoords));
      for (int i = 0; i < 7; i++)
      {
          NS_LOG_UNCOND(bsCoords[i]);
      }

      // choose firsts 7-nBsNoUesAlloc BS coordinates 
      int nodeGroupSize = nUeNodes / (7 - nBsNoUesAlloc);
      int nodeGroupSizeRest = nUeNodes % (7 - nBsNoUesAlloc);
      for (int bsCoordIndex = 0; bsCoordIndex < 7 - nBsNoUesAlloc; bsCoordIndex++)
      {
          // set disc allocator on BS coordinate(isd = isd /2)
          uePositionAlloc->SetX(bsCoords[bsCoordIndex].x);
          uePositionAlloc->SetY(bsCoords[bsCoordIndex].y);
          uePositionAlloc->SetRho(isd / 2);
          uemobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                                      "Mode",
                                      StringValue("Time"),
                                      "Time",
                                      StringValue(std::to_string(timeDirection) + "s"),
                                      "Speed",
                                      PointerValue(speed),
                                      "Bounds",
                                      RectangleValue(Rectangle(0, maxXAxis, 0, maxYAxis)));
          uemobility.SetPositionAllocator(uePositionAlloc);
          for (int ueIndex = nodeGroupSize * bsCoordIndex;
               ueIndex < nodeGroupSize * (bsCoordIndex + 1);
               ueIndex++)
          {
              // NS_LOG_UNCOND(ueNodes.Get(ueIndex)->GetId());
              uemobility.Install(ueNodes.Get(ueIndex));
          }
          // Allocate the remaining UEs along BSs
          if (nodeGroupSizeRest > 0)
          {
              int addedUeIndex = nUeNodes - nodeGroupSizeRest;
              // NS_LOG_UNCOND(ueNodes.Get(addedUeIndex)->GetId());
              uemobility.Install(ueNodes.Get(addedUeIndex));
              nodeGroupSizeRest = nodeGroupSizeRest - 1;
          }
      }
      break;
  }
  default:
      NS_FATAL_ERROR("positionAllocator not recognized" << positionAllocator);
      break;
  }

  // Install mmWave, lte, mc Devices to the nodes
  NetDeviceContainer lteEnbDevs = mmwaveHelper->InstallLteEnbDevice (lteEnbNodes);
  NetDeviceContainer mmWaveEnbDevs = mmwaveHelper->InstallEnbDevice (mmWaveEnbNodes);
  NetDeviceContainer mcUeDevs = mmwaveHelper->InstallMcUeDevice (ueNodes);

  // Install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (mcUeDevs));
  // Assign IP address to UEs, and install applications
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting =
          ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

  // Add X2 interfaces
  mmwaveHelper->AddX2Interface (lteEnbNodes, mmWaveEnbNodes);

  // The hotspot was centred on mmWaveEnbNodes.Get(hotspotCellId - 2), which
  // assumes cell ids are handed out in install order (LTE first). Check it.
  if (hotspotFraction > 0.0 &&
      DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (hotspotCellId - 2))->GetCellId () !=
          hotspotCellId)
    {
      NS_FATAL_ERROR ("hotspot centred on the wrong cell: node index "
                      << hotspotCellId - 2 << " is not cell " << hotspotCellId);
    }

  // Manual attachment
  mmwaveHelper->AttachToClosestEnb (mcUeDevs, mmWaveEnbDevs, lteEnbDevs);

  // Install and start applications
  // On the remoteHost there are TCP and UDP OnOff Applications
  uint16_t portTcp = 50000;
  Address sinkLocalAddressTcp (InetSocketAddress (Ipv4Address::GetAny (), portTcp));
  PacketSinkHelper sinkHelperTcp ("ns3::TcpSocketFactory", sinkLocalAddressTcp);
  AddressValue serverAddressTcp (InetSocketAddress (remoteHostAddr, portTcp));

  uint16_t portUdp = 60000;
  Address sinkLocalAddressUdp (InetSocketAddress (Ipv4Address::GetAny (), portUdp));
  PacketSinkHelper sinkHelperUdp ("ns3::UdpSocketFactory", sinkLocalAddressUdp);
  AddressValue serverAddressUdp (InetSocketAddress (remoteHostAddr, portUdp));

  ApplicationContainer sinkApp;
  sinkApp.Add (sinkHelperTcp.Install (remoteHost));
  sinkApp.Add (sinkHelperUdp.Install (remoteHost));

  // On the UEs there are TCP and UDP clients
  // If needed [Mean=1,Bound=0]
  OnOffHelper clientHelperTcp ("ns3::TcpSocketFactory", Address ());
  clientHelperTcp.SetAttribute ("Remote", serverAddressTcp);
  clientHelperTcp.SetAttribute ("OnTime", StringValue ("ns3::ExponentialRandomVariable"));
  clientHelperTcp.SetAttribute ("OffTime", StringValue ("ns3::ExponentialRandomVariable"));
  clientHelperTcp.SetAttribute ("DataRate", StringValue (dataRate));
  clientHelperTcp.SetAttribute ("PacketSize", UintegerValue (1280));

  // clientHelperTcp150/750 are no longer installed, but keep them: their
  // ExponentialRandomVariables consume RNG streams, and removing them changes
  // every later stream (same-seed runs stop being byte-identical).
  OnOffHelper clientHelperTcp150 ("ns3::TcpSocketFactory", Address ());
  clientHelperTcp150.SetAttribute ("Remote", serverAddressTcp);
  clientHelperTcp150.SetAttribute ("OnTime", StringValue ("ns3::ExponentialRandomVariable"));
  clientHelperTcp150.SetAttribute ("OffTime", StringValue ("ns3::ExponentialRandomVariable"));
  clientHelperTcp150.SetAttribute ("DataRate", StringValue ("150kbps"));
  clientHelperTcp150.SetAttribute ("PacketSize", UintegerValue (1280));

  OnOffHelper clientHelperTcp750 ("ns3::TcpSocketFactory", Address ());
  clientHelperTcp750.SetAttribute ("Remote", serverAddressTcp);
  clientHelperTcp750.SetAttribute ("OnTime", StringValue ("ns3::ExponentialRandomVariable"));
  clientHelperTcp750.SetAttribute ("OffTime", StringValue ("ns3::ExponentialRandomVariable"));
  clientHelperTcp750.SetAttribute ("DataRate", StringValue ("750kbps"));
  clientHelperTcp750.SetAttribute ("PacketSize", UintegerValue (1280));

  OnOffHelper clientHelperUdp ("ns3::UdpSocketFactory", Address ());
  clientHelperUdp.SetAttribute ("Remote", serverAddressUdp);
  clientHelperUdp.SetAttribute ("OnTime", StringValue ("ns3::ExponentialRandomVariable"));
  clientHelperUdp.SetAttribute ("OffTime", StringValue ("ns3::ExponentialRandomVariable"));
  clientHelperUdp.SetAttribute ("DataRate", StringValue (dataRate));
  clientHelperUdp.SetAttribute ("PacketSize", UintegerValue (1280));

  // DOWNLINK bursty flows for trafficModel=3 (see g_burstyDlRate1). The sink
  // lives on the UE and the OnOff source on the remote host, which is the
  // opposite of clientHelperTcp*/clientHelperUdp above. Port 1235 keeps these
  // clear of the full-buffer UDP flows on 1234, so DlE2PdcpStats attributes
  // them to the right bearer and Satisfaction -- which selects UEs by IMSI via
  // MlbZmqEnv._udp_imsis, (imsi-1) % 4 == 0 -- is unaffected.
  uint16_t portUdpDl = 1235;
  PacketSinkHelper sinkHelperUdpDl (
      "ns3::UdpSocketFactory",
      Address (InetSocketAddress (Ipv4Address::GetAny (), portUdpDl)));

  // Mean ON/OFF durations from burstyOnMeanS/burstyOffMeanS (default 1.0 s).
  std::ostringstream burstyOnTimeStr, burstyOffTimeStr;
  burstyOnTimeStr << "ns3::ExponentialRandomVariable[Mean=" << burstyOnMeanS << "]";
  burstyOffTimeStr << "ns3::ExponentialRandomVariable[Mean=" << burstyOffMeanS << "]";
  std::string burstyOnTimeRv = burstyOnTimeStr.str ();
  std::string burstyOffTimeRv = burstyOffTimeStr.str ();
  NS_LOG_UNCOND ("Bursty DL OnTime " << burstyOnTimeRv << " OffTime " << burstyOffTimeRv);

  OnOffHelper clientHelperUdpDl1 ("ns3::UdpSocketFactory", Address ());
  clientHelperUdpDl1.SetAttribute ("OnTime", StringValue (burstyOnTimeRv));
  clientHelperUdpDl1.SetAttribute ("OffTime", StringValue (burstyOffTimeRv));
  clientHelperUdpDl1.SetAttribute ("DataRate", StringValue (burstyDlRate1));
  clientHelperUdpDl1.SetAttribute ("PacketSize", UintegerValue (1280));

  OnOffHelper clientHelperUdpDl2 ("ns3::UdpSocketFactory", Address ());
  clientHelperUdpDl2.SetAttribute ("OnTime", StringValue (burstyOnTimeRv));
  clientHelperUdpDl2.SetAttribute ("OffTime", StringValue (burstyOffTimeRv));
  clientHelperUdpDl2.SetAttribute ("DataRate", StringValue (burstyDlRate2));
  clientHelperUdpDl2.SetAttribute ("PacketSize", UintegerValue (1280));

  OnOffHelper clientHelperUdpDl3 ("ns3::UdpSocketFactory", Address ());
  clientHelperUdpDl3.SetAttribute ("OnTime", StringValue (burstyOnTimeRv));
  clientHelperUdpDl3.SetAttribute ("OffTime", StringValue (burstyOffTimeRv));
  clientHelperUdpDl3.SetAttribute ("DataRate", StringValue (burstyDlRate3));
  clientHelperUdpDl3.SetAttribute ("PacketSize", UintegerValue (1280));

  ApplicationContainer clientApp;
  switch (trafficModel)
    {
      case 0: {
        for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
          {
            // Full traffic
            PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory",
                                                 InetSocketAddress (Ipv4Address::GetAny (), 1234));
            sinkApp.Add (dlPacketSinkHelper.Install (ueNodes.Get (u)));
            UdpClientHelper dlClient (ueIpIface.GetAddress (u), 1234);
            dlClient.SetAttribute ("Interval", TimeValue (MicroSeconds (500)));
            dlClient.SetAttribute ("MaxPackets", UintegerValue (UINT32_MAX));
            dlClient.SetAttribute ("PacketSize", UintegerValue (1280));
            clientApp.Add (dlClient.Install (remoteHost));
          }
      }
      break;

      case 1: {
        for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
          {

            if (u % 2 == 0)
              {
                // Bursty traffic
                if (u % 4 == 0)
                  {
                    clientApp.Add (clientHelperTcp.Install (ueNodes.Get (u)));
                  }
                else
                  {
                    clientApp.Add (clientHelperUdp.Install (ueNodes.Get (u)));
                  }
              }
            else
              {
                // Full traffic
                PacketSinkHelper dlPacketSinkHelper (
                    "ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), 1234));
                sinkApp.Add (dlPacketSinkHelper.Install (ueNodes.Get (u)));
                UdpClientHelper dlClient (ueIpIface.GetAddress (u), 1234);
                dlClient.SetAttribute ("Interval", TimeValue (MicroSeconds (500)));
                dlClient.SetAttribute ("MaxPackets", UintegerValue (UINT32_MAX));
                dlClient.SetAttribute ("PacketSize", UintegerValue (1280));
                clientApp.Add (dlClient.Install (remoteHost));
              }
          }
      }
      break;

      case 2: {
        for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
          {
            // Bursty traffic
            if (u % 2 == 0)
              {
                clientApp.Add (clientHelperTcp.Install (ueNodes.Get (u)));
              }
            else
              {
                clientApp.Add (clientHelperUdp.Install (ueNodes.Get (u)));
              }
          }
      }
      break;

      case 3: { // 25% bursty Full-buffer traffic
                // 25% Bursty traffic with higher application bit-rate averaging around 3 Mbps
                // 25% Bursty traffic with higher application bit-rate averaging around 750 Kbps
                // 25% Bursty traffic with lower application bit-rate averaging around 150 Kbps.
        for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
          {

            if (u % 4 == 0)
              {
                // Full buffer traffic
                PacketSinkHelper dlPacketSinkHelper (
                    "ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), 1234));
                sinkApp.Add (dlPacketSinkHelper.Install (ueNodes.Get (u)));
                UdpClientHelper dlClient (ueIpIface.GetAddress (u), 1234);
                dlClient.SetAttribute ("MaxPackets", UintegerValue (UINT32_MAX));
                dlClient.SetAttribute ("PacketSize", UintegerValue (1280));
                if (configuration == 2)
                  {
                    // Data rate 40 Mbps
                    dlClient.SetAttribute ("Interval", TimeValue (MicroSeconds (250)));
                  }
                else
                  {
                    // Data rate 20.96 Mbps at the default 500 us; tunable via
                    // --udpFullBufferIntervalUs to place offered DL load near
                    // capacity instead of far above it.
                    dlClient.SetAttribute (
                        "Interval", TimeValue (MicroSeconds (udpFullBufferIntervalUs)));
                  }

                clientApp.Add (dlClient.Install (remoteHost));
              }
            else if (u % 4 == 1 || u % 4 == 2 || u % 4 == 3)
              {
                // DOWNLINK bursty UDP: sink on the UE, source on the remote
                // host. Was UPLINK TCP installed on the UE, which no reward
                // term could see -- see g_burstyDlRate1 for the measurements.
                sinkApp.Add (sinkHelperUdpDl.Install (ueNodes.Get (u)));
                AddressValue ueDlAddr (
                    InetSocketAddress (ueIpIface.GetAddress (u), portUdpDl));
                OnOffHelper &dlHelper = (u % 4 == 1)   ? clientHelperUdpDl1
                                        : (u % 4 == 2) ? clientHelperUdpDl2
                                                       : clientHelperUdpDl3;
                dlHelper.SetAttribute ("Remote", ueDlAddr);
                clientApp.Add (dlHelper.Install (remoteHost));
              }
          }
        break;
      }

    default:
      NS_FATAL_ERROR (
          "Traffic model not recognized, the only possible values are [0,1,2,3]. Value passed: "
          << trafficModel);

    }

  // Start applications
  GlobalValue::GetValueByName ("simTime", doubleValue);
  double simTime = doubleValue.Get ();
  sinkApp.Start (Seconds (0));

  clientApp.Start (MilliSeconds (100));
  clientApp.Stop (Seconds (simTime - 0.1));

  int BsStatus[4] = {bsOn, bsIdle, bsSleep, bsOff};

  // bsIdle turn ON the BS like would do bsOn
  // If bsIdle is equal to zero, treat bsIdle as bsOn and put bsOn=0, in this way we are skipping
  // the first part of heuristic 1 and 2 regarding SINR calculus and comparison: through this
  // changing we will give the possibility the cells that are turned OFF, to turn ON again
  if (bsIdle == 0)
  {
      BsStatus[1] = bsOn;
      BsStatus[0] = 0;
  }

  Ptr<EnergyHeuristic> energyHeur=CreateObject<EnergyHeuristic>();
  switch (heuristicType)
    {
      // No heuristc
      case -1: {
        NS_LOG_UNCOND ("Running the scenario with no Energy Heuristic");
      }
      break;

      // Random sleeping
      case 0: {
        for (double i = 0.0; i < simTime; i = i + indicationPeriodicity)
          {
            for (int j = 0; j < nMmWaveEnbNodes; j++)
              {
                Ptr<MmWaveEnbNetDevice> mmdev =
                    DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
                Ptr<LteEnbNetDevice> ltedev = DynamicCast<LteEnbNetDevice> (lteEnbDevs.Get (0));
                Simulator::Schedule (Seconds (i), &EnergyHeuristic::ProbabilityState, energyHeur,
                                     probOn, probIdle, probSleep, probOff, mmdev, ltedev);
              }
          }
      }
      break;

      // Static sleeping
      case 1: {
        for (double i = 0.0; i < simTime; i = i + indicationPeriodicity)
          {
            //If bsOn==0 skip it: we don't need to count the SINR of connected UEs
            for (int j = 0; j < nMmWaveEnbNodes && bsOn!=0; j++)
              {
                Ptr<MmWaveEnbNetDevice> mmdev =
                    DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
                Simulator::Schedule (Seconds (i), &EnergyHeuristic::CountBestUesSinr, energyHeur,
                                     sinrTh, mmdev);
              }
            Ptr<LteEnbNetDevice> ltedev = DynamicCast<LteEnbNetDevice> (lteEnbDevs.Get (0));
            Simulator::Schedule (Seconds (i), &EnergyHeuristic::TurnOnBsSinrPos, energyHeur,
                                 nMmWaveEnbNodes, mmWaveEnbDevs, "static", BsStatus, ltedev);
          }
      }
      break;

      // Dynamic sleeping
      case 2: {

        for (double i = 0.0; i < simTime; i = i + indicationPeriodicity)
          {
            //If bsOn==0 skip it: we don't need to count the SINR of connected UEs
            for (int j = 0; j < nMmWaveEnbNodes && bsOn!=0; j++)
              {
                Ptr<MmWaveEnbNetDevice> mmdev =
                    DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (j));
                Simulator::Schedule (Seconds (i), &EnergyHeuristic::CountBestUesSinr, energyHeur,
                                     sinrTh, mmdev);
              }
            Ptr<LteEnbNetDevice> ltedev = DynamicCast<LteEnbNetDevice> (lteEnbDevs.Get (0));
            Simulator::Schedule (Seconds (i), &EnergyHeuristic::TurnOnBsSinrPos, energyHeur,
                                 nMmWaveEnbNodes, mmWaveEnbDevs, "dynamic", BsStatus, ltedev);
          }
      }
      break;
      default: {
        NS_FATAL_ERROR (
            "Heuristic type not recognized, the only possible values are [-1,0,1,2]. Value passed: "
            << heuristicType);
      }
      break;
    }

  if (enableTraces)
  {
    mmwaveHelper->EnableTraces ();
  }  

  // trick to enable PHY traces for the LTE stack
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  lteHelper->Initialize ();
  lteHelper->EnablePhyTraces ();
  lteHelper->EnableMacTraces ();

  // Since nodes are randomly allocated during each run we always need to print their positions
  PrintGnuplottableUeListToFile ("ues.txt");
  PrintGnuplottableEnbListToFile ("enbs.txt");
  if (logChannelConditions)
    {
      PointerValue condPtr;
      DynamicCast<ThreeGppPropagationLossModel> (mmwaveHelper->GetPathLossModel (0))
          ->GetAttribute ("ChannelConditionModel", condPtr);
      Ptr<ChannelConditionModel> condModel = condPtr.Get<ChannelConditionModel> ();
      NS_ABORT_MSG_IF (!condModel, "logChannelConditions: no channel condition model");
      Simulator::Schedule (MilliSeconds (50), &LogChannelConditions, condModel, ueNodes,
                           mmWaveEnbNodes, controlIntervalCfg, std::string ("channel_conditions.txt"));
    }
  if (hotspotFraction > 0.0)
    {
      // End-of-run positions, to check the hotspot is still there after the walk.
      Simulator::Schedule (Seconds (simTime) - MilliSeconds (1),
                           &PrintGnuplottableUeListToFile, std::string ("ues_end.txt"));
    }
  Ptr<LteEnbNetDevice> ltedev = DynamicCast<LteEnbNetDevice> (lteEnbDevs.Get (0));
  Ptr<LteEnbRrc> lte_rrc = ltedev->GetRrc ();  
  for (double i = 0.0; i < simTime; i = i + indicationPeriodicity){
    Simulator::Schedule (Seconds (i), BsStateTrace,"bsState.txt", ltedev, lte_rrc);
  }

  // =========================================================================
  // ZERO-MQ MARL BRIDGE INITIALIZATION & SCHEDULING
  // =========================================================================
  // Allocate client on the heap so it persists during the entire Simulator::Run()
  // Hook the UE RRC handover traces. Both paths are required: the mmWave one is
  // what CIO actually drives, and the LTE one is kept so an inter-RAT fallback is
  // not silently missed. Connected here, after every device exists, because
  // ConnectFailSafe silently matches nothing if the path is not yet populated.
  // Mirrors mmwave-bearer-stats-connector.cc:369-373.
  Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
                           MakeCallback (&MarlHandoverStartCallback));
  Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/MmWaveUeRrc/HandoverStart",
                           MakeCallback (&MarlHandoverStartCallback));

  std::string zmqEndpoint = "tcp://localhost:" + std::to_string (zmqPort);
  NS_LOG_UNCOND ("MARL bridge endpoint " << zmqEndpoint
                 << " controlInterval " << controlIntervalCfg << " s");
  ns3::ZmqDatabaseClient* zmqClient = new ns3::ZmqDatabaseClient(zmqEndpoint);
  zmqClient->Connect();

  double controlInterval = controlIntervalCfg; // Control interval T_control in seconds

  // Schedule the first MARL step at t = 0.0 using the primary LTE eNodeB device
  // (control target) and the full mmWave eNB container (real per-cell KPI source)
  Ptr<LteEnbNetDevice> primaryLteEnb = lteEnbDevs.Get(0)->GetObject<LteEnbNetDevice>();
  Simulator::Schedule (Seconds (controlPhaseOffsetS),
                       &MarlControlStep,
                       zmqClient,
                       primaryLteEnb,
                       mmWaveEnbDevs,
                       controlInterval);
  // =========================================================================

  bool run = true;
  if (run)
    {
      NS_LOG_UNCOND ("Simulation time is " << simTime << " seconds ");
      Simulator::Stop (Seconds (simTime));
      NS_LOG_INFO ("Run Simulation.");
      Simulator::Run ();
    }

  NS_LOG_INFO (lteHelper);

  // --- Clean up ZeroMQ Client & Simulator ---
  zmqClient->Disconnect();
  delete zmqClient;

  Simulator::Destroy ();
  NS_LOG_INFO ("Done.");
  return 0;
}
