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
 */

#ifndef THREE_GPP_SPECTRUM_PROPAGATION_LOSS_H
#define THREE_GPP_SPECTRUM_PROPAGATION_LOSS_H

#include "ns3/matrix-based-channel-model.h"
#include "ns3/phased-array-spectrum-propagation-loss-model.h"
#include "ns3/random-variable-stream.h"

#include <complex.h>
#include <map>
#include <unordered_map>

// Test class, declared here so the model can befriend it (see the friend
// declaration below). Same pattern as two-ray-spectrum-propagation-loss-model.h.
class ThreeGppDelayPhasorCacheTest;

namespace ns3
{

class NetDevice;

/**
 * \ingroup spectrum
 * \brief 3GPP Spectrum Propagation Loss Model
 *
 * This class models the frequency dependent propagation phenomena in the way
 * described by 3GPP TR 38.901 document. The main method is DoCalcRxPowerSpectralDensity,
 * which takes as input the power spectral density (PSD) of the transmitted signal,
 * the mobility models of the transmitting node and receiving node, and
 * returns the PSD of the received signal.
 *
 * \see MatrixBasedChannelModel
 * \see PhasedArrayModel
 * \see ChannelCondition
 */
class ThreeGppSpectrumPropagationLossModel : public PhasedArraySpectrumPropagationLossModel
{
    // Needs access to m_delayPhasorMap to assert that the cap actually evicted,
    // so the eviction test cannot pass vacuously.
    friend class ::ThreeGppDelayPhasorCacheTest;

  public:
    /**
     * Constructor
     */
    ThreeGppSpectrumPropagationLossModel();

    /**
     * Destructor
     */
    ~ThreeGppSpectrumPropagationLossModel() override;

    void DoDispose() override;

    /**
     * Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    /**
     * Set the channel model object
     * \param channel a pointer to an object implementing the MatrixBasedChannelModel interface
     */
    void SetChannelModel(Ptr<MatrixBasedChannelModel> channel);

    /**
     * Get the channel model object
     * \return a pointer to the object implementing the MatrixBasedChannelModel interface
     */
    Ptr<MatrixBasedChannelModel> GetChannelModel() const;

    /**
     * Sets the value of an attribute belonging to the associated
     * MatrixBasedChannelModel instance
     * \param name name of the attribute
     * \param value the attribute value
     */
    void SetChannelModelAttribute(const std::string& name, const AttributeValue& value);

    /**
     * Returns the value of an attribute belonging to the associated
     * MatrixBasedChannelModel instance
     * \param name name of the attribute
     * \param value where the result should be stored
     */
    void GetChannelModelAttribute(const std::string& name, AttributeValue& value) const;

    /**
     * \brief Computes the received PSD.
     *
     * This function computes the received PSD by applying the 3GPP fast fading
     * model and the beamforming gain.
     * In particular, it retrieves the matrix representing the channel between
     * node a and node b, computes the corresponding long term component, i.e.,
     * the product between the cluster matrices and the TX and RX beamforming
     * vectors (w_rx^T H^n_ab w_tx), and accounts for the Doppler component and
     * the propagation delay.
     * To reduce the computational load, the long term component associated with
     * a certain channel is cached and recomputed only when the channel realization
     * is updated, or when the beamforming vectors change.
     *
     * \param params tx parameters
     * \param a first node mobility model
     * \param b second node mobility model
     * \param aPhasedArrayModel the antenna array of the first node
     * \param bPhasedArrayModel the antenna array of the second node
     * \return the received PSD
     */
    Ptr<SpectrumValue> DoCalcRxPowerSpectralDensity(
        Ptr<const SpectrumSignalParameters> params,
        Ptr<const MobilityModel> a,
        Ptr<const MobilityModel> b,
        Ptr<const PhasedArrayModel> aPhasedArrayModel,
        Ptr<const PhasedArrayModel> bPhasedArrayModel) const override;

  private:
    /**
     * Data structure that stores the long term component for a tx-rx pair
     */
    struct LongTerm : public SimpleRefCount<LongTerm>
    {
        PhasedArrayModel::ComplexVector
            m_longTerm; //!< vector containing the long term component for each cluster
        Ptr<const MatrixBasedChannelModel::ChannelMatrix>
            m_channel; //!< pointer to the channel matrix used to compute the long term
        PhasedArrayModel::ComplexVector
            m_sW; //!< the beamforming vector for the node s used to compute the long term
        PhasedArrayModel::ComplexVector
            m_uW; //!< the beamforming vector for the node u used to compute the long term
    };

    /**
     * Get the operating frequency
     * \return the operating frequency in Hz
     */
    double GetFrequency() const;

    /**
     * Looks for the long term component in m_longTermMap. If found, checks
     * whether it has to be updated. If not found or if it has to be updated,
     * calls the method CalcLongTerm to compute it.
     * \param channelMatrix the channel matrix
     * \param aPhasedArrayModel the antenna array of the tx device
     * \param bPhasedArrayModel the antenna array of the rx device
     * \return vector containing the long term component for each cluster
     */
    PhasedArrayModel::ComplexVector GetLongTerm(
        Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix,
        Ptr<const PhasedArrayModel> aPhasedArrayModel,
        Ptr<const PhasedArrayModel> bPhasedArrayModel) const;
    /**
     * Computes the long term component
     * \param channelMatrix the channel matrix H
     * \param sW the beamforming vector of the s device
     * \param uW the beamforming vector of the u device
     * \return the long term component
     */
    PhasedArrayModel::ComplexVector CalcLongTerm(
        Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix,
        const PhasedArrayModel::ComplexVector& sW,
        const PhasedArrayModel::ComplexVector& uW) const;

    /**
     * Computes the beamforming gain and applies it to the tx PSD
     * \param txPsd the tx PSD
     * \param longTerm the long term component
     * \param channelMatrix The channel matrix structure
     * \param channelParams The channel params structure
     * \param sSpeed speed of the first node
     * \param uSpeed speed of the second node
     * \return the rx PSD
     */
    Ptr<SpectrumValue> CalcBeamformingGain(
        Ptr<SpectrumValue> txPsd,
        PhasedArrayModel::ComplexVector longTerm,
        Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix,
        Ptr<const MatrixBasedChannelModel::ChannelParams> channelParams,
        const Vector& sSpeed,
        const Vector& uSpeed) const;

    /**
     * Cached per-cluster, per-sub-band phase rotations exp(-j*2*pi*f_sb*tau_c)
     * for one tx-rx pair.
     *
     * CalcBeamformingGain applies this factor to every cluster of every
     * sub-band on every transmission, and both of its inputs -- the sub-band
     * centre frequency and the cluster delay -- are fixed until the channel
     * parameters are regenerated (ThreeGppChannelModel::UpdatePeriod). Caching
     * it replaces two transcendental calls per (sub-band, cluster) per
     * transmission with a table lookup and is exact: the stored value is the
     * same std::complex<double> the uncached path computes.
     *
     * The Doppler term is deliberately NOT cached here. It depends on the two
     * nodes' velocities, which change with mobility BETWEEN regenerations, and
     * on the s/u direction of the channel params, neither of which
     * m_generatedTime tracks.
     */
    struct DelayPhasors : public SimpleRefCount<DelayPhasors>
    {
        Time m_generatedTime{Time::Min()}; //!< generation time of the params this was built from
        std::size_t m_numBands{0};         //!< number of sub-bands the table was built for
        uint16_t m_numCluster{0};          //!< number of clusters the table was built for
        SpectrumModelUid_t m_smUid{0};     //!< uid of the spectrum model the table was built for
        std::vector<std::complex<double>> m_phasor; //!< [cluster * m_numBands + band]
    };

    /**
     * Returns the delay-phasor table for the given tx-rx pair, rebuilding it if
     * the channel parameters have been regenerated, the geometry has changed or
     * the caller is using a different spectrum model.
     *
     * The map key is a hint derived from the node pair and the spectrum model
     * uid; correctness does not depend on it being collision-free, because
     * every field the table depends on is stored in the entry and revalidated
     * on each lookup. A collision therefore costs a rebuild, never a wrong
     * answer.
     *
     * \param psd a PSD carrying the sub-band structure the table must match
     * \param channelParams the channel params holding the cluster delays
     * \param numCluster the number of clusters
     * \return the cached table, or nullptr if caching is disabled
     */
    const DelayPhasors* GetDelayPhasors(
        Ptr<const SpectrumValue> psd,
        Ptr<const MatrixBasedChannelModel::ChannelParams> channelParams,
        uint16_t numCluster) const;

    mutable std::unordered_map<uint64_t, Ptr<const LongTerm>>
        m_longTermMap;                           //!< map containing the long term components
    /**
     * Delay-phasor tables, one per (tx-rx pair, spectrum model). Entries are
     * overwritten in place rather than appended, so the footprint is a function
     * of the topology and not of how long the simulation runs. It is cleared on
     * DoDispose, and wholesale whenever inserting or refreshing an entry would
     * take it over m_phasorCacheMaxMB, so a scenario that creates and destroys
     * nodes cannot grow it without bound. The cap is checked only on that
     * insert/refresh path, never on a cache hit, which keeps the hot path to a
     * lookup; and a single table larger than the cap is still stored, so the
     * bound is (cap + one table) rather than a hard ceiling.
     */
    mutable std::unordered_map<uint64_t, Ptr<DelayPhasors>> m_delayPhasorMap;
    mutable std::size_t m_delayPhasorBytes{0};   //!< current size of m_delayPhasorMap in bytes
    bool m_cacheDelayPhasors{true};              //!< whether to cache the delay phasors
    uint32_t m_phasorCacheMaxMB{1024};           //!< hard cap on m_delayPhasorMap, in MiB
    Ptr<MatrixBasedChannelModel> m_channelModel; //!< the model to generate the channel matrix
};
} // namespace ns3

#endif /* THREE_GPP_SPECTRUM_PROPAGATION_LOSS_H */
