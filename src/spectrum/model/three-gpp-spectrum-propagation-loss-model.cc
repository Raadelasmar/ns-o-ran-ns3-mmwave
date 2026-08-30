/*
 * Copyright (c) 2015, NYU WIRELESS, Tandon School of Engineering,
 * New York University
 * Copyright (c) 2019 SIGNET Lab, Department of Information Engineering,
 * University of Padova
 *
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
 */

#include "three-gpp-spectrum-propagation-loss-model.h"

#include "spectrum-signal-parameters.h"
#include "three-gpp-channel-model.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/net-device.h"
#include "ns3/node.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <map>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThreeGppSpectrumPropagationLossModel");

NS_OBJECT_ENSURE_REGISTERED(ThreeGppSpectrumPropagationLossModel);

ThreeGppSpectrumPropagationLossModel::ThreeGppSpectrumPropagationLossModel()
{
    NS_LOG_FUNCTION(this);
}

ThreeGppSpectrumPropagationLossModel::~ThreeGppSpectrumPropagationLossModel()
{
    NS_LOG_FUNCTION(this);
}

void
ThreeGppSpectrumPropagationLossModel::DoDispose()
{
    m_longTermMap.clear();
    m_delayPhasorMap.clear();
    m_delayPhasorBytes = 0;
    m_channelModel->Dispose();
    m_channelModel = nullptr;
}

TypeId
ThreeGppSpectrumPropagationLossModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThreeGppSpectrumPropagationLossModel")
            .SetParent<PhasedArraySpectrumPropagationLossModel>()
            .SetGroupName("Spectrum")
            .AddConstructor<ThreeGppSpectrumPropagationLossModel>()
            .AddAttribute(
                "ChannelModel",
                "The channel model. It needs to implement the MatrixBasedChannelModel interface",
                StringValue("ns3::ThreeGppChannelModel"),
                MakePointerAccessor(&ThreeGppSpectrumPropagationLossModel::SetChannelModel,
                                    &ThreeGppSpectrumPropagationLossModel::GetChannelModel),
                MakePointerChecker<MatrixBasedChannelModel>())
            .AddAttribute(
                "CacheDelayPhasors",
                "Cache the per-cluster, per-sub-band phase rotation "
                "exp(-j*2*pi*f_sb*tau_c) applied in CalcBeamformingGain. Both of its "
                "inputs are constant until the channel parameters are regenerated, so "
                "caching it is exact and produces bit-identical results while removing "
                "two transcendental calls per sub-band per cluster per transmission.",
                BooleanValue(true),
                MakeBooleanAccessor(
                    &ThreeGppSpectrumPropagationLossModel::m_cacheDelayPhasors),
                MakeBooleanChecker())
            .AddAttribute(
                "DelayPhasorCacheMaxMB",
                "Hard cap on the delay-phasor cache, in MiB. The cache is normally "
                "bounded by the topology because entries are overwritten in place, so "
                "this only binds in a scenario that keeps creating new node pairs; "
                "when it does, the whole cache is dropped and rebuilt on demand. "
                "Measured usage is about 110 MB for a 7-cell, 35-UE mmWave scenario, and it "
                "scales with the number of tx-rx pairs.",
                UintegerValue(1024),
                MakeUintegerAccessor(
                    &ThreeGppSpectrumPropagationLossModel::m_phasorCacheMaxMB),
                MakeUintegerChecker<uint32_t>(1));
    return tid;
}

void
ThreeGppSpectrumPropagationLossModel::SetChannelModel(Ptr<MatrixBasedChannelModel> channel)
{
    m_channelModel = channel;
}

Ptr<MatrixBasedChannelModel>
ThreeGppSpectrumPropagationLossModel::GetChannelModel() const
{
    return m_channelModel;
}

double
ThreeGppSpectrumPropagationLossModel::GetFrequency() const
{
    DoubleValue freq;
    m_channelModel->GetAttribute("Frequency", freq);
    return freq.Get();
}

void
ThreeGppSpectrumPropagationLossModel::SetChannelModelAttribute(const std::string& name,
                                                               const AttributeValue& value)
{
    m_channelModel->SetAttribute(name, value);
}

void
ThreeGppSpectrumPropagationLossModel::GetChannelModelAttribute(const std::string& name,
                                                               AttributeValue& value) const
{
    m_channelModel->GetAttribute(name, value);
}

PhasedArrayModel::ComplexVector
ThreeGppSpectrumPropagationLossModel::CalcLongTerm(
    Ptr<const MatrixBasedChannelModel::ChannelMatrix> params,
    const PhasedArrayModel::ComplexVector& sW,
    const PhasedArrayModel::ComplexVector& uW) const
{
    NS_LOG_FUNCTION(this);

    size_t uAntennaNum = uW.GetSize();
    size_t sAntennaNum = sW.GetSize();

    NS_ASSERT(uAntennaNum == params->m_channel.GetNumRows());
    NS_ASSERT(sAntennaNum == params->m_channel.GetNumCols());

    NS_LOG_DEBUG("CalcLongTerm with " << uAntennaNum << " u antenna elements and " << sAntennaNum
                                      << " s antenna elements.");
    // store the long term part to reduce computation load
    // only the small scale fading needs to be updated if the large scale parameters and antenna
    // weights remain unchanged. here we calculate long term uW * Husn * sW, the result is an array
    // of values per cluster
    return params->m_channel.MultiplyByLeftAndRightMatrix(uW.Transpose(), sW);
}

const ThreeGppSpectrumPropagationLossModel::DelayPhasors*
ThreeGppSpectrumPropagationLossModel::GetDelayPhasors(
    Ptr<const SpectrumValue> psd,
    Ptr<const MatrixBasedChannelModel::ChannelParams> channelParams,
    uint16_t numCluster) const
{
    if (!m_cacheDelayPhasors)
    {
        return nullptr;
    }

    Ptr<const SpectrumModel> sm = psd->GetSpectrumModel();
    const std::size_t numBands = sm->GetNumBands();
    const SpectrumModelUid_t smUid = sm->GetUid();

    // A hint, not an identity: every field the table depends on is revalidated
    // below, so a key collision costs a rebuild rather than a wrong answer.
    const uint64_t key =
        MatrixBasedChannelModel::GetKey(channelParams->m_nodeIds.first,
                                        channelParams->m_nodeIds.second) *
            0x9E3779B97F4A7C15ULL +
        smUid;

    Ptr<DelayPhasors> entry;
    auto it = m_delayPhasorMap.find(key);
    if (it != m_delayPhasorMap.end())
    {
        entry = it->second;
        if (entry->m_generatedTime == channelParams->m_generatedTime &&
            entry->m_numBands == numBands && entry->m_numCluster == numCluster &&
            entry->m_smUid == smUid)
        {
            return PeekPointer(entry);
        }
        m_delayPhasorBytes -= entry->m_phasor.size() * sizeof(std::complex<double>);
    }

    const std::size_t wantBytes =
        static_cast<std::size_t>(numCluster) * numBands * sizeof(std::complex<double>);

    // Entries are overwritten in place, so this only binds for a scenario that
    // keeps introducing new node pairs. Drop everything rather than carry a
    // half-evicted map; the tables rebuild lazily on the next transmission.
    if (m_delayPhasorBytes + wantBytes >
        static_cast<std::size_t>(m_phasorCacheMaxMB) * 1024 * 1024)
    {
        NS_LOG_LOGIC("delay-phasor cache over " << m_phasorCacheMaxMB << " MiB, clearing");
        m_delayPhasorMap.clear();
        m_delayPhasorBytes = 0;
        entry = nullptr;
    }

    if (!entry)
    {
        entry = Create<DelayPhasors>();
        m_delayPhasorMap[key] = entry;
    }

    entry->m_generatedTime = channelParams->m_generatedTime;
    entry->m_numBands = numBands;
    entry->m_numCluster = numCluster;
    entry->m_smUid = smUid;
    entry->m_phasor.assign(static_cast<std::size_t>(numCluster) * numBands,
                           std::complex<double>(0.0, 0.0));
    m_delayPhasorBytes += wantBytes;

    for (uint16_t cIndex = 0; cIndex < numCluster; cIndex++)
    {
        std::size_t band = 0;
        for (auto bit = psd->ConstBandsBegin(); bit != psd->ConstBandsEnd(); ++bit, ++band)
        {
            // Identical expression, and identical operand order, to the
            // uncached branch in CalcBeamformingGain.
            double delay = -2 * M_PI * (*bit).fc * (channelParams->m_delay[cIndex]);
            entry->m_phasor[static_cast<std::size_t>(cIndex) * numBands + band] =
                std::complex<double>(cos(delay), sin(delay));
        }
    }

    return PeekPointer(entry);
}

Ptr<SpectrumValue>
ThreeGppSpectrumPropagationLossModel::CalcBeamformingGain(
    Ptr<SpectrumValue> txPsd,
    PhasedArrayModel::ComplexVector longTerm,
    Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix,
    Ptr<const MatrixBasedChannelModel::ChannelParams> channelParams,
    const ns3::Vector& sSpeed,
    const ns3::Vector& uSpeed) const
{
    NS_LOG_FUNCTION(this);

    Ptr<SpectrumValue> tempPsd = Copy<SpectrumValue>(txPsd);

    // channel[cluster][rx][tx]
    uint16_t numCluster = channelMatrix->m_channel.GetNumPages();

    // compute the doppler term
    // NOTE the update of Doppler is simplified by only taking the center angle of
    // each cluster in to consideration.
    double slotTime = Simulator::Now().GetSeconds();
    double factor = 2 * M_PI * slotTime * GetFrequency() / 3e8;
    PhasedArrayModel::ComplexVector doppler(numCluster);

    // The following asserts might seem paranoic, but it is important to
    // make sure that all the structures that are passed to this function
    // are of the correct dimensions before using the operator [].
    // If you dont understand the comment read about the difference of .at()
    // and [] operators, ...
    NS_ASSERT(numCluster <= channelParams->m_alpha.size());
    NS_ASSERT(numCluster <= channelParams->m_D.size());
    NS_ASSERT(numCluster <= channelParams->m_angle[MatrixBasedChannelModel::ZOA_INDEX].size());
    NS_ASSERT(numCluster <= channelParams->m_angle[MatrixBasedChannelModel::ZOD_INDEX].size());
    NS_ASSERT(numCluster <= channelParams->m_angle[MatrixBasedChannelModel::AOA_INDEX].size());
    NS_ASSERT(numCluster <= channelParams->m_angle[MatrixBasedChannelModel::AOD_INDEX].size());
    NS_ASSERT(numCluster <= longTerm.GetSize());

    // check if channelParams structure is generated in direction s-to-u or u-to-s
    bool isSameDirection = (channelParams->m_nodeIds == channelMatrix->m_nodeIds);

    MatrixBasedChannelModel::DoubleVector zoa;
    MatrixBasedChannelModel::DoubleVector zod;
    MatrixBasedChannelModel::DoubleVector aoa;
    MatrixBasedChannelModel::DoubleVector aod;

    // if channel params is generated in the same direction in which we
    // generate the channel matrix, angles and zenith od departure and arrival are ok,
    // just set them to corresponding variable that will be used for the generation
    // of channel matrix, otherwise we need to flip angles and zeniths of departure and arrival
    if (isSameDirection)
    {
        zoa = channelParams->m_angle[MatrixBasedChannelModel::ZOA_INDEX];
        zod = channelParams->m_angle[MatrixBasedChannelModel::ZOD_INDEX];
        aoa = channelParams->m_angle[MatrixBasedChannelModel::AOA_INDEX];
        aod = channelParams->m_angle[MatrixBasedChannelModel::AOD_INDEX];
    }
    else
    {
        zod = channelParams->m_angle[MatrixBasedChannelModel::ZOA_INDEX];
        zoa = channelParams->m_angle[MatrixBasedChannelModel::ZOD_INDEX];
        aod = channelParams->m_angle[MatrixBasedChannelModel::AOA_INDEX];
        aoa = channelParams->m_angle[MatrixBasedChannelModel::AOD_INDEX];
    }

    for (uint16_t cIndex = 0; cIndex < numCluster; cIndex++)
    {
        // Compute alpha and D as described in 3GPP TR 37.885 v15.3.0, Sec. 6.2.3
        // These terms account for an additional Doppler contribution due to the
        // presence of moving objects in the surrounding environment, such as in
        // vehicular scenarios.
        // This contribution is applied only to the delayed (reflected) paths and
        // must be properly configured by setting the value of
        // m_vScatt, which is defined as "maximum speed of the vehicle in the
        // layout".
        // By default, m_vScatt is set to 0, so there is no additional Doppler
        // contribution.

        double alpha = channelParams->m_alpha[cIndex];
        double D = channelParams->m_D[cIndex];

        // cluster angle angle[direction][n], where direction = 0(aoa), 1(zoa).
        double tempDoppler =
            factor * ((sin(zoa[cIndex] * M_PI / 180) * cos(aoa[cIndex] * M_PI / 180) * uSpeed.x +
                       sin(zoa[cIndex] * M_PI / 180) * sin(aoa[cIndex] * M_PI / 180) * uSpeed.y +
                       cos(zoa[cIndex] * M_PI / 180) * uSpeed.z) +
                      (sin(zod[cIndex] * M_PI / 180) * cos(aod[cIndex] * M_PI / 180) * sSpeed.x +
                       sin(zod[cIndex] * M_PI / 180) * sin(aod[cIndex] * M_PI / 180) * sSpeed.y +
                       cos(zod[cIndex] * M_PI / 180) * sSpeed.z) +
                      2 * alpha * D);
        doppler[cIndex] = std::complex<double>(cos(tempDoppler), sin(tempDoppler));
    }

    NS_ASSERT(numCluster <= doppler.GetSize());

    // apply the doppler term and the propagation delay to the long term component
    // to obtain the beamforming gain
    // The per-(sub-band, cluster) phase rotation below depends only on the
    // sub-band centre frequency and the cluster delay, both fixed until the
    // channel params are regenerated. When CacheDelayPhasors is on it comes
    // from a table instead of two transcendental calls per iteration; the
    // arithmetic that follows is unchanged either way.
    const DelayPhasors* phasors = GetDelayPhasors(tempPsd, channelParams, numCluster);
    const std::size_t numBands = phasors ? phasors->m_numBands : 0;

    auto vit = tempPsd->ValuesBegin();      // psd iterator
    auto sbit = tempPsd->ConstBandsBegin(); // band iterator
    std::size_t bandIdx = 0;                // index of the current sub-band
    while (vit != tempPsd->ValuesEnd())
    {
        if ((*vit) != 0.00)
        {
            std::complex<double> subsbandGain(0.0, 0.0);
            double fsb = (*sbit).fc; // center frequency of the sub-band
            for (uint16_t cIndex = 0; cIndex < numCluster; cIndex++)
            {
                std::complex<double> phase;
                if (phasors)
                {
                    phase = phasors->m_phasor[static_cast<std::size_t>(cIndex) * numBands +
                                              bandIdx];
                }
                else
                {
                    double delay = -2 * M_PI * fsb * (channelParams->m_delay[cIndex]);
                    phase = std::complex<double>(cos(delay), sin(delay));
                }
                subsbandGain = subsbandGain + longTerm[cIndex] * doppler[cIndex] * phase;
            }
            *vit = (*vit) * (norm(subsbandGain));
        }
        vit++;
        sbit++;
        bandIdx++;
    }
    return tempPsd;
}

PhasedArrayModel::ComplexVector
ThreeGppSpectrumPropagationLossModel::GetLongTerm(
    Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix,
    Ptr<const PhasedArrayModel> aPhasedArrayModel,
    Ptr<const PhasedArrayModel> bPhasedArrayModel) const
{
    PhasedArrayModel::ComplexVector
        longTerm; // vector containing the long term component for each cluster

    // check if the channel matrix was generated considering a as the s-node and
    // b as the u-node or vice-versa
    PhasedArrayModel::ComplexVector sW;
    PhasedArrayModel::ComplexVector uW;
    if (!channelMatrix->IsReverse(aPhasedArrayModel->GetId(), bPhasedArrayModel->GetId()))
    {
        sW = aPhasedArrayModel->GetBeamformingVector();
        uW = bPhasedArrayModel->GetBeamformingVector();
    }
    else
    {
        sW = bPhasedArrayModel->GetBeamformingVector();
        uW = aPhasedArrayModel->GetBeamformingVector();
    }

    bool update = false;   // indicates whether the long term has to be updated
    bool notFound = false; // indicates if the long term has not been computed yet

    // compute the long term key, the key is unique for each tx-rx pair
    uint64_t longTermId =
        MatrixBasedChannelModel::GetKey(aPhasedArrayModel->GetId(), bPhasedArrayModel->GetId());

    // look for the long term in the map and check if it is valid
    if (m_longTermMap.find(longTermId) != m_longTermMap.end())
    {
        NS_LOG_DEBUG("found the long term component in the map");
        longTerm = m_longTermMap[longTermId]->m_longTerm;

        // check if the channel matrix has been updated
        // or the s beam has been changed
        // or the u beam has been changed
        update = (m_longTermMap[longTermId]->m_channel->m_generatedTime !=
                      channelMatrix->m_generatedTime ||
                  m_longTermMap[longTermId]->m_sW != sW || m_longTermMap[longTermId]->m_uW != uW);
    }
    else
    {
        NS_LOG_DEBUG("long term component NOT found");
        notFound = true;
    }

    if (update || notFound)
    {
        NS_LOG_DEBUG("compute the long term");
        // compute the long term component
        longTerm = CalcLongTerm(channelMatrix, sW, uW);

        // store the long term
        Ptr<LongTerm> longTermItem = Create<LongTerm>();
        longTermItem->m_longTerm = longTerm;
        longTermItem->m_channel = channelMatrix;
        longTermItem->m_sW = sW;
        longTermItem->m_uW = uW;

        m_longTermMap[longTermId] = longTermItem;
    }

    return longTerm;
}

Ptr<SpectrumValue>
ThreeGppSpectrumPropagationLossModel::DoCalcRxPowerSpectralDensity(
    Ptr<const SpectrumSignalParameters> params,
    Ptr<const MobilityModel> a,
    Ptr<const MobilityModel> b,
    Ptr<const PhasedArrayModel> aPhasedArrayModel,
    Ptr<const PhasedArrayModel> bPhasedArrayModel) const
{
    NS_LOG_FUNCTION(this);
    uint32_t aId = a->GetObject<Node>()->GetId(); // id of the node a
    uint32_t bId = b->GetObject<Node>()->GetId(); // id of the node b

    NS_ASSERT(aId != bId);
    NS_ASSERT_MSG(a->GetDistanceFrom(b) > 0.0,
                  "The position of a and b devices cannot be the same");

    Ptr<SpectrumValue> rxPsd = Copy<SpectrumValue>(params->psd);

    // retrieve the antenna of device a
    NS_ASSERT_MSG(aPhasedArrayModel, "Antenna not found for node " << aId);
    NS_LOG_DEBUG("a node " << a->GetObject<Node>() << " antenna " << aPhasedArrayModel);

    // retrieve the antenna of the device b
    NS_ASSERT_MSG(bPhasedArrayModel, "Antenna not found for device " << bId);
    NS_LOG_DEBUG("b node " << bId << " antenna " << bPhasedArrayModel);

    Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix =
        m_channelModel->GetChannel(a, b, aPhasedArrayModel, bPhasedArrayModel);
    Ptr<const MatrixBasedChannelModel::ChannelParams> channelParams =
        m_channelModel->GetParams(a, b);

    // retrieve the long term component
    PhasedArrayModel::ComplexVector longTerm =
        GetLongTerm(channelMatrix, aPhasedArrayModel, bPhasedArrayModel);

    // apply the beamforming gain
    rxPsd = CalcBeamformingGain(rxPsd,
                                longTerm,
                                channelMatrix,
                                channelParams,
                                a->GetVelocity(),
                                b->GetVelocity());

    return rxPsd;
}

} // namespace ns3
