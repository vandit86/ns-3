/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013 Budiarto Herman
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
 * Author: Budiarto Herman <budiarto.herman@magister.fi>
 *
 */

#ifndef CV2X_LTE_HANDOVER_MANAGEMENT_SAP_H
#define CV2X_LTE_HANDOVER_MANAGEMENT_SAP_H

#include <ns3/cv2x_lte-rrc-sap.h>

namespace ns3 {


/**
 * \brief Service Access Point (SAP) offered by the handover algorithm instance
 *        to the eNodeB RRC instance.
 *
 * This is the *Handover Management SAP Provider*, i.e., the part of the SAP
 * that contains the handover algorithm methods called by the eNodeB RRC
 * instance.
 */
class cv2x_LteHandoverManagementSapProvider
{
public:
  virtual ~cv2x_LteHandoverManagementSapProvider ();

  /**
   * \brief Send a UE measurement report to handover algorithm.
   * \param rnti Radio Network Temporary Identity, an integer identifying the UE
   *             where the report originates from
   * \param measResults a single report of one measurement identity
   *
   * The received measurement report is a result of the UE measurement
   * configuration previously configured by calling
   * cv2x_LteHandoverManagementSapUser::AddUeMeasReportConfigForHandover. The report
   * may be stored and utilised for the purpose of making handover decision.
   */
  virtual void ReportUeMeas (uint16_t rnti,
                             cv2x_LteRrcSap::MeasResults measResults) = 0;

}; // end of class cv2x_LteHandoverManagementSapProvider


/**
 * \brief Service Access Point (SAP) offered by the eNodeB RRC instance to the
 *        handover algorithm instance.
 *
 * This is the *Handover Management SAP User*, i.e., the part of the SAP that
 * contains the eNodeB RRC methods called by the handover algorithm instance.
 */
class cv2x_LteHandoverManagementSapUser
{
public:
  virtual ~cv2x_LteHandoverManagementSapUser ();

  /**
   * \brief Request a certain reporting configuration to be fulfilled by the UEs
   *        attached to the eNodeB entity.
   * \param reportConfig the UE measurement reporting configuration
   * \return the measurement identity associated with this newly added
   *         reporting configuration
   *
   * The eNodeB RRC entity is expected to configure the same reporting
   * configuration in each of the attached UEs. When later in the simulation a
   * UE measurement report is received from a UE as a result of this
   * configuration, the eNodeB RRC entity shall forward this report to the
   * handover algorithm through the cv2x_LteHandoverManagementSapProvider::ReportUeMeas
   * SAP function.
   *
   * \note This function is only valid before the simulation begins.
   */
  virtual uint8_t AddUeMeasReportConfigForHandover (cv2x_LteRrcSap::ReportConfigEutra reportConfig) = 0;

  /**
   * \brief Instruct the eNodeB RRC entity to prepare a handover.
   * \param rnti Radio Network Temporary Identity, an integer identifying the
   *             UE which shall perform the handover
   * \param targetCellId the cell ID of the target eNodeB
   *
   * This function is used by the handover algorithm entity when a handover
   * decision has been reached.
   *
   * The process to produce the decision is up to the implementation of handover
   * algorithm. It is typically based on the reported UE measurements, which are
   * received through the cv2x_LteHandoverManagementSapProvider::ReportUeMeas function.
   */
  virtual void TriggerHandover (uint16_t rnti, uint16_t targetCellId) = 0;

}; // end of class cv2x_LteHandoverManagementSapUser



/**
 * \brief Template for the implementation of the cv2x_LteHandoverManagementSapProvider
 *        as a member of an owner class of type C to which all methods are
 *        forwarded.
 */
template <class C>
class cv2x_MemberLteHandoverManagementSapProvider : public cv2x_LteHandoverManagementSapProvider
{
public:
  /**
   * Constructor
   *
   * \param owner the owner class
   */
  cv2x_MemberLteHandoverManagementSapProvider (C* owner);

  // inherited from LteHandoverManagemenrSapProvider
  virtual void ReportUeMeas (uint16_t rnti, cv2x_LteRrcSap::MeasResults measResults);

private:
  cv2x_MemberLteHandoverManagementSapProvider ();
  C* m_owner; ///< the owner class

}; // end of class cv2x_MemberLteHandoverManagementSapProvider


template <class C>
cv2x_MemberLteHandoverManagementSapProvider<C>::cv2x_MemberLteHandoverManagementSapProvider (C* owner)
  : m_owner (owner)
{
}


template <class C>
void
cv2x_MemberLteHandoverManagementSapProvider<C>::ReportUeMeas (uint16_t rnti, cv2x_LteRrcSap::MeasResults measResults)
{
  m_owner->DoReportUeMeas (rnti, measResults);
}



/**
 * \brief Template for the implementation of the cv2x_LteHandoverManagementSapUser
 *        as a member of an owner class of type C to which all methods are
 *        forwarded.
 */
template <class C>
class cv2x_MemberLteHandoverManagementSapUser : public cv2x_LteHandoverManagementSapUser
{
public:
  /**
   * Constructor
   *
   * \param owner the owner class
   */
  cv2x_MemberLteHandoverManagementSapUser (C* owner);

  // inherited from cv2x_LteHandoverManagementSapUser
  virtual uint8_t AddUeMeasReportConfigForHandover (cv2x_LteRrcSap::ReportConfigEutra reportConfig);
  virtual void TriggerHandover (uint16_t rnti, uint16_t targetCellId);

private:
  cv2x_MemberLteHandoverManagementSapUser ();
  C* m_owner; ///< the owner class

}; // end of class cv2x_MemberLteAnrSapUser


template <class C>
cv2x_MemberLteHandoverManagementSapUser<C>::cv2x_MemberLteHandoverManagementSapUser (C* owner)
  : m_owner (owner)
{
}


template <class C>
uint8_t
cv2x_MemberLteHandoverManagementSapUser<C>::AddUeMeasReportConfigForHandover (cv2x_LteRrcSap::ReportConfigEutra reportConfig)
{
  return m_owner->DoAddUeMeasReportConfigForHandover (reportConfig);
}


template <class C>
void
cv2x_MemberLteHandoverManagementSapUser<C>::TriggerHandover (uint16_t rnti, uint16_t targetCellId)
{
  return m_owner->DoTriggerHandover (rnti, targetCellId);
}


} // end of namespace ns3


#endif /* CV2X_LTE_HANDOVER_MANAGEMENT_SAP_H */
