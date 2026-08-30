/*
 * Copyright (c) 2019 SIGNET Lab, Department of Information Engineering,
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
 */

#include "ns3/abort.h"
#include "ns3/angles.h"
#include "ns3/channel-condition-model.h"
#include "ns3/config.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/double.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/log.h"
#include "ns3/node-container.h"
#include "ns3/pointer.h"
#include "ns3/simple-net-device.h"
#include "ns3/simulator.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/three-gpp-channel-model.h"
#include "ns3/three-gpp-spectrum-propagation-loss-model.h"
#include "ns3/uinteger.h"
#include "ns3/uniform-planar-array.h"
#include "ns3/wifi-spectrum-value-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThreeGppChannelTestSuite");

/**
 * \ingroup spectrum-tests
 *
 * Test case for the ThreeGppChannelModel class.
 * 1) check if the channel matrix has the correct dimensions
 * 2) check if the channel matrix is correctly normalized
 */
class ThreeGppChannelMatrixComputationTest : public TestCase
{
  public:
    /**
     * Constructor
     */
    ThreeGppChannelMatrixComputationTest();

    /**
     * Destructor
     */
    ~ThreeGppChannelMatrixComputationTest() override;

  private:
    /**
     * Build the test scenario
     */
    void DoRun() override;

    /**
     * Compute the Frobenius norm of the channel matrix and stores it in m_normVector
     * \param channelModel the ThreeGppChannelModel object used to generate the channel matrix
     * \param txMob the mobility model of the first node
     * \param rxMob the mobility model of the second node
     * \param txAntenna the antenna object associated to the first node
     * \param rxAntenna the antenna object associated to the second node
     */
    void DoComputeNorm(Ptr<ThreeGppChannelModel> channelModel,
                       Ptr<MobilityModel> txMob,
                       Ptr<MobilityModel> rxMob,
                       Ptr<PhasedArrayModel> txAntenna,
                       Ptr<PhasedArrayModel> rxAntenna);

    std::vector<double> m_normVector; //!< each element is the norm of a channel realization
};

ThreeGppChannelMatrixComputationTest::ThreeGppChannelMatrixComputationTest()
    : TestCase("Check the dimensions and the norm of the channel matrix")
{
}

ThreeGppChannelMatrixComputationTest::~ThreeGppChannelMatrixComputationTest()
{
}

void
ThreeGppChannelMatrixComputationTest::DoComputeNorm(Ptr<ThreeGppChannelModel> channelModel,
                                                    Ptr<MobilityModel> txMob,
                                                    Ptr<MobilityModel> rxMob,
                                                    Ptr<PhasedArrayModel> txAntenna,
                                                    Ptr<PhasedArrayModel> rxAntenna)
{
    uint64_t txAntennaElements = txAntenna->GetNumberOfElements();
    uint64_t rxAntennaElements = rxAntenna->GetNumberOfElements();

    Ptr<const ThreeGppChannelModel::ChannelMatrix> channelMatrix =
        channelModel->GetChannel(txMob, rxMob, txAntenna, rxAntenna);

    double channelNorm = 0;
    uint16_t numTotalClusters = channelMatrix->m_channel.GetNumPages();
    for (uint16_t cIndex = 0; cIndex < numTotalClusters; cIndex++)
    {
        double clusterNorm = 0;
        for (uint64_t sIndex = 0; sIndex < txAntennaElements; sIndex++)
        {
            for (uint64_t uIndex = 0; uIndex < rxAntennaElements; uIndex++)
            {
                clusterNorm +=
                    std::pow(std::abs(channelMatrix->m_channel(uIndex, sIndex, cIndex)), 2);
            }
        }
        channelNorm += clusterNorm;
    }
    m_normVector.push_back(channelNorm);
}

void
ThreeGppChannelMatrixComputationTest::DoRun()
{
    // Build the scenario for the test
    uint8_t txAntennaElements[]{2, 2}; // tx antenna dimensions
    uint8_t rxAntennaElements[]{2, 2}; // rx antenna dimensions
    uint32_t updatePeriodMs = 100;     // update period in ms

    // create the channel condition model
    Ptr<ChannelConditionModel> channelConditionModel =
        CreateObject<NeverLosChannelConditionModel>();

    // create the ThreeGppChannelModel object used to generate the channel matrix
    Ptr<ThreeGppChannelModel> channelModel = CreateObject<ThreeGppChannelModel>();
    channelModel->SetAttribute("Frequency", DoubleValue(60.0e9));
    channelModel->SetAttribute("Scenario", StringValue("RMa"));
    channelModel->SetAttribute("ChannelConditionModel", PointerValue(channelConditionModel));
    channelModel->SetAttribute("UpdatePeriod", TimeValue(MilliSeconds(updatePeriodMs - 1)));

    // create the tx and rx nodes
    NodeContainer nodes;
    nodes.Create(2);

    // create the tx and rx devices
    Ptr<SimpleNetDevice> txDev = CreateObject<SimpleNetDevice>();
    Ptr<SimpleNetDevice> rxDev = CreateObject<SimpleNetDevice>();

    // associate the nodes and the devices
    nodes.Get(0)->AddDevice(txDev);
    txDev->SetNode(nodes.Get(0));
    nodes.Get(1)->AddDevice(rxDev);
    rxDev->SetNode(nodes.Get(1));

    // create the tx and rx mobility models and set their positions
    Ptr<MobilityModel> txMob = CreateObject<ConstantPositionMobilityModel>();
    txMob->SetPosition(Vector(0.0, 0.0, 10.0));
    Ptr<MobilityModel> rxMob = CreateObject<ConstantPositionMobilityModel>();
    rxMob->SetPosition(Vector(100.0, 0.0, 10.0));

    // associate the nodes and the mobility models
    nodes.Get(0)->AggregateObject(txMob);
    nodes.Get(1)->AggregateObject(rxMob);

    // create the tx and rx antennas and set the their dimensions
    Ptr<PhasedArrayModel> txAntenna = CreateObjectWithAttributes<UniformPlanarArray>(
        "NumColumns",
        UintegerValue(txAntennaElements[0]),
        "NumRows",
        UintegerValue(txAntennaElements[1]),
        "AntennaElement",
        PointerValue(CreateObject<IsotropicAntennaModel>()));
    Ptr<PhasedArrayModel> rxAntenna = CreateObjectWithAttributes<UniformPlanarArray>(
        "NumColumns",
        UintegerValue(rxAntennaElements[0]),
        "NumRows",
        UintegerValue(rxAntennaElements[1]),
        "AntennaElement",
        PointerValue(CreateObject<IsotropicAntennaModel>()));

    // generate the channel matrix
    Ptr<const ThreeGppChannelModel::ChannelMatrix> channelMatrix =
        channelModel->GetChannel(txMob, rxMob, txAntenna, rxAntenna);

    // check the channel matrix dimensions, expected H[cluster][rx][tx]
    NS_TEST_ASSERT_MSG_EQ(
        channelMatrix->m_channel.GetNumCols(),
        txAntennaElements[0] * txAntennaElements[1],
        "The third dimension of H should be equal to the number of tx antenna elements");
    NS_TEST_ASSERT_MSG_EQ(
        channelMatrix->m_channel.GetNumRows(),
        rxAntennaElements[0] * rxAntennaElements[1],
        "The second dimension of H should be equal to the number of rx antenna elements");

    // test if the channel matrix is correctly generated
    uint16_t numIt = 1000;
    for (uint16_t i = 0; i < numIt; i++)
    {
        Simulator::Schedule(MilliSeconds(updatePeriodMs * i),
                            &ThreeGppChannelMatrixComputationTest::DoComputeNorm,
                            this,
                            channelModel,
                            txMob,
                            rxMob,
                            txAntenna,
                            rxAntenna);
    }

    Simulator::Run();

    // compute the sample mean
    double sampleMean = 0;
    for (auto i : m_normVector)
    {
        sampleMean += i;
    }
    sampleMean /= numIt;

    // compute the sample standard deviation
    double sampleStd = 0;
    for (auto i : m_normVector)
    {
        sampleStd += ((i - sampleMean) * (i - sampleMean));
    }
    sampleStd = std::sqrt(sampleStd / (numIt - 1));

    // perform the one sample t-test with a significance level of 0.05 to test
    // the hypothesis "E [|H|^2] = M*N, where |H| indicates the Frobenius norm of
    // H, M is the number of transmit antenna elements, and N is the number of
    // the receive antenna elements"
    double t = (sampleMean - txAntennaElements[0] * txAntennaElements[1] * rxAntennaElements[0] *
                                 rxAntennaElements[1]) /
               (sampleMean / std::sqrt(numIt));

    // Using a significance level of 0.05, we reject the null hypothesis if |t| is
    // greater than the critical value from a t-distribution with df = numIt-1
    NS_TEST_ASSERT_MSG_EQ_TOL(
        std::abs(t),
        0,
        1.65,
        "We reject the hypothesis E[|H|^2] = M*N with a significance level of 0.05");

    Simulator::Destroy();
}

/**
 * \ingroup spectrum-tests
 *
 * Test case for the ThreeGppChannelModel class.
 * It checks if the channel realizations are correctly updated during the
 * simulation.
 */
class ThreeGppChannelMatrixUpdateTest : public TestCase
{
  public:
    /**
     * Constructor
     */
    ThreeGppChannelMatrixUpdateTest();

    /**
     * Destructor
     */
    ~ThreeGppChannelMatrixUpdateTest() override;

  private:
    /**
     * Build the test scenario
     */
    void DoRun() override;

    /**
     * This method is used to schedule the channel matrix computation at different
     * time instants and to check if it correctly updated
     * \param channelModel the ThreeGppChannelModel object used to generate the channel matrix
     * \param txMob the mobility model of the first node
     * \param rxMob the mobility model of the second node
     * \param txAntenna the antenna object associated to the first node
     * \param rxAntenna the antenna object associated to the second node
     * \param update whether if the channel matrix should be updated or not
     */
    void DoGetChannel(Ptr<ThreeGppChannelModel> channelModel,
                      Ptr<MobilityModel> txMob,
                      Ptr<MobilityModel> rxMob,
                      Ptr<PhasedArrayModel> txAntenna,
                      Ptr<PhasedArrayModel> rxAntenna,
                      bool update);

    Ptr<const ThreeGppChannelModel::ChannelMatrix>
        m_currentChannel; //!< used by DoGetChannel to store the current channel matrix
};

ThreeGppChannelMatrixUpdateTest::ThreeGppChannelMatrixUpdateTest()
    : TestCase("Check if the channel realizations are correctly updated during the simulation")
{
}

ThreeGppChannelMatrixUpdateTest::~ThreeGppChannelMatrixUpdateTest()
{
}

void
ThreeGppChannelMatrixUpdateTest::DoGetChannel(Ptr<ThreeGppChannelModel> channelModel,
                                              Ptr<MobilityModel> txMob,
                                              Ptr<MobilityModel> rxMob,
                                              Ptr<PhasedArrayModel> txAntenna,
                                              Ptr<PhasedArrayModel> rxAntenna,
                                              bool update)
{
    // retrieve the channel matrix
    Ptr<const ThreeGppChannelModel::ChannelMatrix> channelMatrix =
        channelModel->GetChannel(txMob, rxMob, txAntenna, rxAntenna);

    if (!m_currentChannel)
    {
        // this is the first time we compute the channel matrix, we initialize
        // m_currentChannel
        m_currentChannel = channelMatrix;
    }
    else
    {
        // compare the old and the new channel matrices
        NS_TEST_ASSERT_MSG_EQ((m_currentChannel != channelMatrix),
                              update,
                              Simulator::Now().GetMilliSeconds()
                                  << " The channel matrix is not correctly updated");
    }
}

void
ThreeGppChannelMatrixUpdateTest::DoRun()
{
    // Build the scenario for the test

    uint8_t txAntennaElements[]{2, 2}; // tx antenna dimensions
    uint8_t rxAntennaElements[]{4, 4}; // rx antenna dimensions
    uint32_t updatePeriodMs = 100;     // update period in ms

    // create the channel condition model
    Ptr<ChannelConditionModel> channelConditionModel =
        CreateObject<AlwaysLosChannelConditionModel>();

    // create the ThreeGppChannelModel object used to generate the channel matrix
    Ptr<ThreeGppChannelModel> channelModel = CreateObject<ThreeGppChannelModel>();
    channelModel->SetAttribute("Frequency", DoubleValue(60.0e9));
    channelModel->SetAttribute("Scenario", StringValue("UMa"));
    channelModel->SetAttribute("ChannelConditionModel", PointerValue(channelConditionModel));
    channelModel->SetAttribute("UpdatePeriod", TimeValue(MilliSeconds(updatePeriodMs)));

    // create the tx and rx nodes
    NodeContainer nodes;
    nodes.Create(2);

    // create the tx and rx devices
    Ptr<SimpleNetDevice> txDev = CreateObject<SimpleNetDevice>();
    Ptr<SimpleNetDevice> rxDev = CreateObject<SimpleNetDevice>();

    // associate the nodes and the devices
    nodes.Get(0)->AddDevice(txDev);
    txDev->SetNode(nodes.Get(0));
    nodes.Get(1)->AddDevice(rxDev);
    rxDev->SetNode(nodes.Get(1));

    // create the tx and rx mobility models and set their positions
    Ptr<MobilityModel> txMob = CreateObject<ConstantPositionMobilityModel>();
    txMob->SetPosition(Vector(0.0, 0.0, 10.0));
    Ptr<MobilityModel> rxMob = CreateObject<ConstantPositionMobilityModel>();
    rxMob->SetPosition(Vector(100.0, 0.0, 1.6));

    // associate the nodes and the mobility models
    nodes.Get(0)->AggregateObject(txMob);
    nodes.Get(1)->AggregateObject(rxMob);

    // create the tx and rx antennas and set the their dimensions
    Ptr<PhasedArrayModel> txAntenna = CreateObjectWithAttributes<UniformPlanarArray>(
        "NumColumns",
        UintegerValue(txAntennaElements[0]),
        "NumRows",
        UintegerValue(txAntennaElements[1]),
        "AntennaElement",
        PointerValue(CreateObject<IsotropicAntennaModel>()));
    Ptr<PhasedArrayModel> rxAntenna = CreateObjectWithAttributes<UniformPlanarArray>(
        "NumColumns",
        UintegerValue(rxAntennaElements[0]),
        "NumRows",
        UintegerValue(rxAntennaElements[1]),
        "AntennaElement",
        PointerValue(CreateObject<IsotropicAntennaModel>()));

    // check if the channel matrix is correctly updated

    // compute the channel matrix for the first time
    uint32_t firstTimeMs =
        1; // time instant at which the channel matrix is generated for the first time
    Simulator::Schedule(MilliSeconds(firstTimeMs),
                        &ThreeGppChannelMatrixUpdateTest::DoGetChannel,
                        this,
                        channelModel,
                        txMob,
                        rxMob,
                        txAntenna,
                        rxAntenna,
                        true);

    // call GetChannel before the update period is exceeded, the channel matrix
    // should not be updated
    Simulator::Schedule(MilliSeconds(firstTimeMs + updatePeriodMs / 2),
                        &ThreeGppChannelMatrixUpdateTest::DoGetChannel,
                        this,
                        channelModel,
                        txMob,
                        rxMob,
                        txAntenna,
                        rxAntenna,
                        false);

    // call GetChannel when the update period is exceeded, the channel matrix
    // should be recomputed
    Simulator::Schedule(MilliSeconds(firstTimeMs + updatePeriodMs + 1),
                        &ThreeGppChannelMatrixUpdateTest::DoGetChannel,
                        this,
                        channelModel,
                        txMob,
                        rxMob,
                        txAntenna,
                        rxAntenna,
                        true);

    Simulator::Run();
    Simulator::Destroy();
}

/**
 * \ingroup spectrum-tests
 * \brief A structure that holds the parameters for the function
 * CheckLongTermUpdate. In this way the problem with the limited
 * number of parameters of method Schedule is avoided.
 */
struct CheckLongTermUpdateParams
{
    Ptr<ThreeGppSpectrumPropagationLossModel>
        lossModel; //!< the ThreeGppSpectrumPropagationLossModel object used to compute the rx PSD
    Ptr<SpectrumSignalParameters> txParams; //!< the params of the tx signal
    Ptr<MobilityModel> txMob;               //!< the mobility model of the tx device
    Ptr<MobilityModel> rxMob;               //!< the mobility model of the rx device
    Ptr<SpectrumValue> rxPsdOld;            //!< the previously received PSD
    Ptr<PhasedArrayModel> txAntenna;        //!< the antenna array of the tx device
    Ptr<PhasedArrayModel> rxAntenna;        //!< the antenna array of the rx device
};

/**
 * \ingroup spectrum-tests
 *
 * Test case for the ThreeGppSpectrumPropagationLossModelTest class.
 * 1) checks if the long term components for the direct and the reverse link
 *    are the same
 * 2) checks if the long term component is updated when changing the beamforming
 *    vectors
 * 3) checks if the long term is updated when changing the channel matrix
 */
class ThreeGppSpectrumPropagationLossModelTest : public TestCase
{
  public:
    /**
     * Constructor
     */
    ThreeGppSpectrumPropagationLossModelTest();

    /**
     * Destructor
     */
    ~ThreeGppSpectrumPropagationLossModelTest() override;

  private:
    /**
     * Build the test scenario
     */
    void DoRun() override;

    /**
     * Points the beam of thisDevice towards otherDevice
     * \param thisDevice the device to configure
     * \param thisAntenna the antenna object associated to thisDevice
     * \param otherDevice the device to communicate with
     * \param otherAntenna the antenna object associated to otherDevice
     */
    void DoBeamforming(Ptr<NetDevice> thisDevice,
                       Ptr<PhasedArrayModel> thisAntenna,
                       Ptr<NetDevice> otherDevice,
                       Ptr<PhasedArrayModel> otherAntenna);

    /**
     * Test of the long term component is correctly updated when the channel
     * matrix is recomputed
     * \param params a structure that contains the set of parameters needed by CheckLongTermUpdate
     * in order to perform calculations
     */
    void CheckLongTermUpdate(const CheckLongTermUpdateParams& params);

    /**
     * Checks if two PSDs are equal
     * \param first the first PSD
     * \param second the second PSD
     * \return true if first and second are equal, false otherwise
     */
    static bool ArePsdEqual(Ptr<SpectrumValue> first, Ptr<SpectrumValue> second);
};

ThreeGppSpectrumPropagationLossModelTest::ThreeGppSpectrumPropagationLossModelTest()
    : TestCase("Test case for the ThreeGppSpectrumPropagationLossModel class")
{
}

ThreeGppSpectrumPropagationLossModelTest::~ThreeGppSpectrumPropagationLossModelTest()
{
}

void
ThreeGppSpectrumPropagationLossModelTest::DoBeamforming(Ptr<NetDevice> thisDevice,
                                                        Ptr<PhasedArrayModel> thisAntenna,
                                                        Ptr<NetDevice> otherDevice,
                                                        Ptr<PhasedArrayModel> otherAntenna)
{
    Vector aPos = thisDevice->GetNode()->GetObject<MobilityModel>()->GetPosition();
    Vector bPos = otherDevice->GetNode()->GetObject<MobilityModel>()->GetPosition();

    // compute the azimuth and the elevation angles
    Angles completeAngle(bPos, aPos);

    PhasedArrayModel::ComplexVector antennaWeights =
        thisAntenna->GetBeamformingVector(completeAngle);
    thisAntenna->SetBeamformingVector(antennaWeights);
}

bool
ThreeGppSpectrumPropagationLossModelTest::ArePsdEqual(Ptr<SpectrumValue> first,
                                                      Ptr<SpectrumValue> second)
{
    bool ret = true;
    for (uint8_t i = 0; i < first->GetSpectrumModel()->GetNumBands(); i++)
    {
        if ((*first)[i] != (*second)[i])
        {
            ret = false;
            continue;
        }
    }
    return ret;
}

void
ThreeGppSpectrumPropagationLossModelTest::CheckLongTermUpdate(
    const CheckLongTermUpdateParams& params)
{
    Ptr<SpectrumValue> rxPsdNew = params.lossModel->DoCalcRxPowerSpectralDensity(params.txParams,
                                                                                 params.txMob,
                                                                                 params.rxMob,
                                                                                 params.txAntenna,
                                                                                 params.rxAntenna);
    NS_TEST_ASSERT_MSG_EQ(ArePsdEqual(params.rxPsdOld, rxPsdNew),
                          false,
                          "The long term is not updated when the channel matrix is recomputed");
}

void
ThreeGppSpectrumPropagationLossModelTest::DoRun()
{
    // Build the scenario for the test
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(100)));

    uint8_t txAntennaElements[]{4, 4}; // tx antenna dimensions
    uint8_t rxAntennaElements[]{4, 4}; // rx antenna dimensions

    // create the ChannelConditionModel object to be used to retrieve the
    // channel condition
    Ptr<ChannelConditionModel> condModel = CreateObject<AlwaysLosChannelConditionModel>();

    // create the ThreeGppSpectrumPropagationLossModel object, set frequency,
    // scenario and channel condition model to be used
    Ptr<ThreeGppSpectrumPropagationLossModel> lossModel =
        CreateObject<ThreeGppSpectrumPropagationLossModel>();
    lossModel->SetChannelModelAttribute("Frequency", DoubleValue(2.4e9));
    lossModel->SetChannelModelAttribute("Scenario", StringValue("UMa"));
    lossModel->SetChannelModelAttribute(
        "ChannelConditionModel",
        PointerValue(condModel)); // create the ThreeGppChannelModel object used to generate the
                                  // channel matrix

    // create the tx and rx nodes
    NodeContainer nodes;
    nodes.Create(2);

    // create the tx and rx devices
    Ptr<SimpleNetDevice> txDev = CreateObject<SimpleNetDevice>();
    Ptr<SimpleNetDevice> rxDev = CreateObject<SimpleNetDevice>();

    // associate the nodes and the devices
    nodes.Get(0)->AddDevice(txDev);
    txDev->SetNode(nodes.Get(0));
    nodes.Get(1)->AddDevice(rxDev);
    rxDev->SetNode(nodes.Get(1));

    // create the tx and rx mobility models and set their positions
    Ptr<MobilityModel> txMob = CreateObject<ConstantPositionMobilityModel>();
    txMob->SetPosition(Vector(0.0, 0.0, 10.0));
    Ptr<MobilityModel> rxMob = CreateObject<ConstantPositionMobilityModel>();
    rxMob->SetPosition(
        Vector(15.0, 0.0, 10.0)); // in this position the channel condition is always LOS

    // associate the nodes and the mobility models
    nodes.Get(0)->AggregateObject(txMob);
    nodes.Get(1)->AggregateObject(rxMob);

    // create the tx and rx antennas and set the their dimensions
    Ptr<PhasedArrayModel> txAntenna = CreateObjectWithAttributes<UniformPlanarArray>(
        "NumColumns",
        UintegerValue(txAntennaElements[0]),
        "NumRows",
        UintegerValue(txAntennaElements[1]),
        "AntennaElement",
        PointerValue(CreateObject<IsotropicAntennaModel>()));
    Ptr<PhasedArrayModel> rxAntenna = CreateObjectWithAttributes<UniformPlanarArray>(
        "NumColumns",
        UintegerValue(rxAntennaElements[0]),
        "NumRows",
        UintegerValue(rxAntennaElements[1]),
        "AntennaElement",
        PointerValue(CreateObject<IsotropicAntennaModel>()));

    // set the beamforming vectors
    DoBeamforming(txDev, txAntenna, rxDev, rxAntenna);
    DoBeamforming(rxDev, rxAntenna, txDev, txAntenna);

    // create the tx psd
    WifiSpectrumValue5MhzFactory sf;
    double txPower = 0.1; // Watts
    uint32_t channelNumber = 1;
    Ptr<SpectrumValue> txPsd = sf.CreateTxPowerSpectralDensity(txPower, channelNumber);
    Ptr<SpectrumSignalParameters> txParams = Create<SpectrumSignalParameters>();
    txParams->psd = txPsd->Copy();

    // compute the rx psd
    Ptr<SpectrumValue> rxPsdOld =
        lossModel->DoCalcRxPowerSpectralDensity(txParams, txMob, rxMob, txAntenna, rxAntenna);

    // 1) check that the rx PSD is equal for both the direct and the reverse channel
    Ptr<SpectrumValue> rxPsdNew =
        lossModel->DoCalcRxPowerSpectralDensity(txParams, rxMob, txMob, rxAntenna, txAntenna);
    NS_TEST_ASSERT_MSG_EQ(ArePsdEqual(rxPsdOld, rxPsdNew),
                          true,
                          "The long term for the direct and the reverse channel are different");

    // 2) check if the long term is updated when changing the BF vector
    // change the position of the rx device and recompute the beamforming vectors
    rxMob->SetPosition(Vector(10.0, 5.0, 10.0));
    PhasedArrayModel::ComplexVector txBfVector = txAntenna->GetBeamformingVector();
    txBfVector[0] = std::complex<double>(0.0, 0.0);
    txAntenna->SetBeamformingVector(txBfVector);

    rxPsdNew =
        lossModel->DoCalcRxPowerSpectralDensity(txParams, rxMob, txMob, rxAntenna, txAntenna);
    NS_TEST_ASSERT_MSG_EQ(ArePsdEqual(rxPsdOld, rxPsdNew),
                          false,
                          "Changing the BF vectors the rx PSD does not change");

    // update rxPsdOld
    rxPsdOld = rxPsdNew;

    // 3) check if the long term is updated when the channel matrix is recomputed
    CheckLongTermUpdateParams
        params{lossModel, txParams, txMob, rxMob, rxPsdOld, txAntenna, rxAntenna};
    Simulator::Schedule(MilliSeconds(101),
                        &ThreeGppSpectrumPropagationLossModelTest::CheckLongTermUpdate,
                        this,
                        params);

    Simulator::Run();
    Simulator::Destroy();
}

/**
 * \ingroup spectrum-tests
 *
 * Tests ThreeGppSpectrumPropagationLossModel's delay-phasor cache.
 *
 * CalcBeamformingGain applies exp(-j*2*pi*f_sb*tau_c) to every cluster of every
 * sub-band on every transmission. Both inputs are constant until the channel
 * params are regenerated, so the CacheDelayPhasors attribute tabulates it. The
 * cache must be exact, and the DelayPhasorCacheMaxMB safety valve must stay
 * exact even while it is evicting on every call.
 *
 * Three configurations are compared on identical inputs:
 *   a) caching off -- the reference
 *   b) caching on, cap never reached -- the normal path
 *   c) caching on, cap deliberately smaller than a single table -- so every
 *      lookup for a second tx-rx pair drops the whole map and rebuilds
 *
 * All three must produce bit-identical PSDs. The map size is asserted directly
 * (via friendship) so that (c) cannot pass without the eviction having fired.
 */
class ThreeGppDelayPhasorCacheTest : public TestCase
{
  public:
    ThreeGppDelayPhasorCacheTest();
    ~ThreeGppDelayPhasorCacheTest() override;

  private:
    void DoRun() override;

    /**
     * Point one antenna at another node
     * \param thisMob mobility of the node holding thisAntenna
     * \param otherMob mobility of the node being pointed at
     * \param thisAntenna the antenna to steer
     */
    static void Steer(Ptr<MobilityModel> thisMob,
                      Ptr<MobilityModel> otherMob,
                      Ptr<PhasedArrayModel> thisAntenna);

    /**
     * Exact, band-count-safe PSD comparison
     * \param first the first PSD
     * \param second the second PSD
     * \return true if every band is bit-identical
     */
    static bool PsdIdentical(Ptr<const SpectrumValue> first, Ptr<const SpectrumValue> second);
};

ThreeGppDelayPhasorCacheTest::ThreeGppDelayPhasorCacheTest()
    : TestCase("ThreeGppSpectrumPropagationLossModel delay-phasor cache is exact, including "
               "when the size cap is evicting")
{
}

ThreeGppDelayPhasorCacheTest::~ThreeGppDelayPhasorCacheTest()
{
}

void
ThreeGppDelayPhasorCacheTest::Steer(Ptr<MobilityModel> thisMob,
                                    Ptr<MobilityModel> otherMob,
                                    Ptr<PhasedArrayModel> thisAntenna)
{
    Angles completeAngle(otherMob->GetPosition(), thisMob->GetPosition());
    thisAntenna->SetBeamformingVector(thisAntenna->GetBeamformingVector(completeAngle));
}

bool
ThreeGppDelayPhasorCacheTest::PsdIdentical(Ptr<const SpectrumValue> first,
                                           Ptr<const SpectrumValue> second)
{
    if (first->GetSpectrumModel()->GetNumBands() != second->GetSpectrumModel()->GetNumBands())
    {
        return false;
    }
    for (std::size_t i = 0; i < first->GetSpectrumModel()->GetNumBands(); i++)
    {
        if ((*first)[i] != (*second)[i])
        {
            return false;
        }
    }
    return true;
}

void
ThreeGppDelayPhasorCacheTest::DoRun()
{
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(100)));

    Ptr<ChannelConditionModel> condModel = CreateObject<AlwaysLosChannelConditionModel>();
    Ptr<ThreeGppSpectrumPropagationLossModel> lossModel =
        CreateObject<ThreeGppSpectrumPropagationLossModel>();
    lossModel->SetChannelModelAttribute("Frequency", DoubleValue(2.4e9));
    lossModel->SetChannelModelAttribute("Scenario", StringValue("UMa"));
    lossModel->SetChannelModelAttribute("ChannelConditionModel", PointerValue(condModel));

    // Two INDEPENDENT tx-rx pairs, each with its own antennas, so the
    // beamforming vectors stay fixed for the whole test and the only thing that
    // varies between phases is the cache configuration.
    NodeContainer nodes;
    nodes.Create(4);
    std::vector<Ptr<MobilityModel>> mob(4);
    std::vector<Ptr<PhasedArrayModel>> ant(4);
    const double xPos[4]{0.0, 15.0, 0.0, -15.0};
    const double yPos[4]{0.0, 0.0, 40.0, 40.0};
    for (uint32_t i = 0; i < 4; i++)
    {
        Ptr<SimpleNetDevice> dev = CreateObject<SimpleNetDevice>();
        nodes.Get(i)->AddDevice(dev);
        dev->SetNode(nodes.Get(i));
        mob[i] = CreateObject<ConstantPositionMobilityModel>();
        mob[i]->SetPosition(Vector(xPos[i], yPos[i], 10.0));
        nodes.Get(i)->AggregateObject(mob[i]);
        ant[i] = CreateObjectWithAttributes<UniformPlanarArray>(
            "NumColumns",
            UintegerValue(2),
            "NumRows",
            UintegerValue(2),
            "AntennaElement",
            PointerValue(CreateObject<IsotropicAntennaModel>()));
    }
    Steer(mob[0], mob[1], ant[0]);
    Steer(mob[1], mob[0], ant[1]);
    Steer(mob[2], mob[3], ant[2]);
    Steer(mob[3], mob[2], ant[3]);

    // A deliberately wide spectrum model: one cached table is
    // numCluster * numBands * sizeof(complex<double>), which at 16384 bands is
    // megabytes, so the 1 MiB cap below is smaller than a single table and the
    // eviction branch is reached on the very first lookup of the second pair.
    const std::size_t numBands = 16384;
    Bands bands;
    for (std::size_t i = 0; i < numBands; i++)
    {
        BandInfo bi;
        bi.fl = 2.4e9 + i * 1.0e4;
        bi.fc = bi.fl + 5.0e3;
        bi.fh = bi.fl + 1.0e4;
        bands.push_back(bi);
    }
    Ptr<SpectrumModel> sm = Create<SpectrumModel>(bands);
    Ptr<SpectrumValue> txPsd = Create<SpectrumValue>(sm);
    (*txPsd) = 0.1 / numBands; // Watts spread over the band
    Ptr<SpectrumSignalParameters> txParams = Create<SpectrumSignalParameters>();
    txParams->psd = txPsd->Copy();

    // (a) reference: caching off
    lossModel->SetAttribute("CacheDelayPhasors", BooleanValue(false));
    Ptr<SpectrumValue> refA =
        lossModel->DoCalcRxPowerSpectralDensity(txParams, mob[0], mob[1], ant[0], ant[1]);
    Ptr<SpectrumValue> refB =
        lossModel->DoCalcRxPowerSpectralDensity(txParams, mob[2], mob[3], ant[2], ant[3]);
    NS_TEST_ASSERT_MSG_EQ(lossModel->m_delayPhasorMap.empty(),
                          true,
                          "Nothing should be cached while CacheDelayPhasors is false");

    // (b) caching on, cap never reached. Same channel matrices, same beamforming
    // vectors, same simulated time -- only the phasor source differs.
    lossModel->SetAttribute("CacheDelayPhasors", BooleanValue(true));
    lossModel->SetAttribute("DelayPhasorCacheMaxMB", UintegerValue(1024));
    for (uint32_t rep = 0; rep < 3; rep++)
    {
        Ptr<SpectrumValue> gotA =
            lossModel->DoCalcRxPowerSpectralDensity(txParams, mob[0], mob[1], ant[0], ant[1]);
        Ptr<SpectrumValue> gotB =
            lossModel->DoCalcRxPowerSpectralDensity(txParams, mob[2], mob[3], ant[2], ant[3]);
        NS_TEST_ASSERT_MSG_EQ(PsdIdentical(refA, gotA),
                              true,
                              "Cached rx PSD differs from the uncached one (pair A, rep "
                                  << rep << ")");
        NS_TEST_ASSERT_MSG_EQ(PsdIdentical(refB, gotB),
                              true,
                              "Cached rx PSD differs from the uncached one (pair B, rep "
                                  << rep << ")");
    }
    NS_TEST_ASSERT_MSG_EQ(lossModel->m_delayPhasorMap.size(),
                          2,
                          "Both tx-rx pairs should be resident when the cap is generous");

    // (c) caching on, cap smaller than one table.
    //
    // The cap is consulted when an entry is inserted or refreshed, never on a
    // hit -- a valid entry returns before any size check, which is what keeps
    // the hot path cheap. So the map has to be cold for the eviction branch to
    // be reachable at all; leaving (b)'s two valid entries in place made an
    // earlier version of this test pass without ever evicting.
    lossModel->m_delayPhasorMap.clear();
    lossModel->m_delayPhasorBytes = 0;
    lossModel->SetAttribute("DelayPhasorCacheMaxMB", UintegerValue(1));
    for (uint32_t rep = 0; rep < 3; rep++)
    {
        Ptr<SpectrumValue> gotA =
            lossModel->DoCalcRxPowerSpectralDensity(txParams, mob[0], mob[1], ant[0], ant[1]);
        NS_TEST_ASSERT_MSG_EQ(lossModel->m_delayPhasorMap.size(),
                              1,
                              "The cap should have evicted the other pair (rep " << rep << ")");
        Ptr<SpectrumValue> gotB =
            lossModel->DoCalcRxPowerSpectralDensity(txParams, mob[2], mob[3], ant[2], ant[3]);
        NS_TEST_ASSERT_MSG_EQ(lossModel->m_delayPhasorMap.size(),
                              1,
                              "The cap should have evicted the other pair (rep " << rep << ")");
        NS_TEST_ASSERT_MSG_EQ(PsdIdentical(refA, gotA),
                              true,
                              "Evicting rebuilt a different phasor table (pair A, rep "
                                  << rep << ")");
        NS_TEST_ASSERT_MSG_EQ(PsdIdentical(refB, gotB),
                              true,
                              "Evicting rebuilt a different phasor table (pair B, rep "
                                  << rep << ")");
    }

    // A single table larger than the whole cap is still stored rather than
    // refused, so the cap is a soft bound of (cap + one table). That is
    // deliberate -- the alternative is a transmission that cannot be computed --
    // and it is asserted here so the behaviour is not changed by accident.
    NS_TEST_ASSERT_MSG_GT(lossModel->m_delayPhasorBytes,
                          1024 * 1024,
                          "One oversized table should still be resident after eviction");

    Simulator::Destroy();
}

/**
 * \ingroup spectrum-tests
 *
 * Test suite for the ThreeGppChannelModel class
 */
class ThreeGppChannelTestSuite : public TestSuite
{
  public:
    /**
     * Constructor
     */
    ThreeGppChannelTestSuite();
};

ThreeGppChannelTestSuite::ThreeGppChannelTestSuite()
    : TestSuite("three-gpp-channel", UNIT)
{
    AddTestCase(new ThreeGppChannelMatrixComputationTest, TestCase::QUICK);
    AddTestCase(new ThreeGppChannelMatrixUpdateTest, TestCase::QUICK);
    AddTestCase(new ThreeGppSpectrumPropagationLossModelTest, TestCase::QUICK);
    AddTestCase(new ThreeGppDelayPhasorCacheTest, TestCase::QUICK);
}

/// Static variable for test initialization
static ThreeGppChannelTestSuite myTestSuite;
