/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014 Piotr Gawlowicz
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
 * Author: Piotr Gawlowicz <gawlowicz.p@gmail.com>
 *
 */

#ifndef CV2X_LTE_FFR_SAP_H
#define CV2X_LTE_FFR_SAP_H

#include <ns3/cv2x_lte-rrc-sap.h>
#include <ns3/cv2x_ff-mac-sched-sap.h>
#include <map>

namespace ns3 {

/**
 * \brief Service Access Point (SAP) offered by the Frequency Reuse algorithm
 *        instance to the MAC Scheduler instance.
 *
 * This is the *cv2x_LteFfrSapProvider*, i.e., the part of the SAP
 * that contains the Frequency Reuse algorithm methods called by the MAC Scheduler
 * instance.
 */
class cv2x_LteFfrSapProvider
{
public:
  virtual ~cv2x_LteFfrSapProvider ();

  /**
    * \brief Get vector of available RBG in DL for this Cell
    * \return vector of size (m_dlBandwidth/RbgSize); false indicates
    *                   that RBG is free to use, true otherwise
    *
    * This function is called by MAC Scheduler in the beginning of DL
    * scheduling process. Frequency Reuse Algorithm based on its policy
    * generates vector of RBG which can be used and which can not be used
    * by Scheduler to schedule transmission.
    */
  virtual std::vector <bool> GetAvailableDlRbg () = 0;

  /**
   * \brief Check if UE can be served on i-th RB in DL
   * \param i RBG ID
   * \param rnti Radio Network Temporary Identity, an integer identifying the UE
   *             where the report originates from
   * \return true if UE can be served on i-th RB, false otherwise
   *
   * This function is called by MAC Scheduler during DL scheduling process
   * to check if UE is allowed to be served with i-th RBG. Frequency Reuse
   * Algorithm based on its policy decides if RBG is allowed to UE.
   * If yes, Scheduler will try to allocate this RBG for UE, if not this UE
   * will not be served with this RBG.
   */
  virtual bool IsDlRbgAvailableForUe (int i, uint16_t rnti) = 0;

  /**
   * \brief Get vector of available RB in UL for this Cell
   * \return vector of size m_ulBandwidth; false indicates
   *                    that RB is free to use, true otherwise
   *
   * This function is called by MAC Scheduler in the beginning of UL
   * scheduling process. Frequency Reuse Algorithm based on its policy
   * generates vector of RB which can be used and which can not be used
   * by Scheduler to schedule transmission.
   */
  virtual std::vector <bool> GetAvailableUlRbg () = 0;

  /**
   * \brief Check if UE can be served on i-th RB in UL
   * \param i RB ID
   * \param rnti Radio Network Temporary Identity, an integer identifying the UE
   *             where the report originates from
   * \return true if UE can be served on i-th RB, false otherwise
   *
   * This function is called by MAC Scheduler during UL scheduling process
   * to check if UE is allowed to be served with i-th RB. Frequency Reuse
   * Algorithm based on its policy decides if RB is allowed to UE.
   * If yes, Scheduler will try to allocate this RB for UE, if not this UE
   * will not be served with this RB.
   */
  virtual bool IsUlRbgAvailableForUe (int i, uint16_t rnti) = 0;

  /**
   * \brief ReportDlCqiInfo
   * \param params the struct cv2x_FfMacSchedSapProvider::SchedDlCqiInfoReqParameters
   */
  virtual void ReportDlCqiInfo (const struct cv2x_FfMacSchedSapProvider::SchedDlCqiInfoReqParameters& params) = 0;

  /**
   * \brief ReportUlCqiInfo
   * \param params the struct cv2x_FfMacSchedSapProvider::SchedUlCqiInfoReqParameters
   */
  virtual void ReportUlCqiInfo (const struct cv2x_FfMacSchedSapProvider::SchedUlCqiInfoReqParameters& params) = 0;

  /**
   * \brief ReportUlCqiInfo
   * \param ulCqiMap the UL CQI map
   */
  virtual void ReportUlCqiInfo ( std::map <uint16_t, std::vector <double> > ulCqiMap ) = 0;

  /**
   * \brief GetTpc
   * \param rnti the RNTI
   * \returns the TCP
   */
  virtual uint8_t GetTpc (uint16_t rnti) = 0;

  /**
   * \brief Get the minimum continuous Ul bandwidth
   * \returns the minimum continuous UL bandwidth
   */
  virtual uint8_t GetMinContinuousUlBandwidth () = 0;
}; // end of class cv2x_LteFfrSapProvider


/**
 * \brief Service Access Point (SAP) offered by the eNodeB RRC instance to the
 *        Frequency Reuse algorithm instance.
 *
 * This is the *cv2x_LteFfrSapUser*, i.e., the part of the SAP that
 * contains the MAC Scheduler methods called by the Frequency Reuse algorithm instance.
 */
class cv2x_LteFfrSapUser
{
public:
  virtual ~cv2x_LteFfrSapUser ();

}; // end of class cv2x_LteFfrSapUser


/**
 * \brief Template for the implementation of the cv2x_LteFfrSapProvider
 *        as a member of an owner class of type C to which all methods are
 *        forwarded.
 */
template <class C>
class cv2x_MemberLteFfrSapProvider : public cv2x_LteFfrSapProvider
{
public:
  /**
   * Constructor
   * 
   * \param owner the owner class
   */
  cv2x_MemberLteFfrSapProvider (C* owner);

  // inherited from cv2x_LteFfrSapProvider
  virtual std::vector <bool> GetAvailableDlRbg ();
  virtual bool IsDlRbgAvailableForUe (int i, uint16_t rnti);
  virtual std::vector <bool> GetAvailableUlRbg ();
  virtual bool IsUlRbgAvailableForUe (int i, uint16_t rnti);
  virtual void ReportDlCqiInfo (const struct cv2x_FfMacSchedSapProvider::SchedDlCqiInfoReqParameters& params);
  virtual void ReportUlCqiInfo (const struct cv2x_FfMacSchedSapProvider::SchedUlCqiInfoReqParameters& params);
  virtual void ReportUlCqiInfo ( std::map <uint16_t, std::vector <double> > ulCqiMap );
  virtual uint8_t GetTpc (uint16_t rnti);
  virtual uint8_t GetMinContinuousUlBandwidth ();
private:
  cv2x_MemberLteFfrSapProvider ();
  C* m_owner; ///< the owner class

}; // end of class cv2x_MemberLteFfrSapProvider


template <class C>
cv2x_MemberLteFfrSapProvider<C>::cv2x_MemberLteFfrSapProvider (C* owner)
  : m_owner (owner)
{
}


template <class C>
std::vector <bool>
cv2x_MemberLteFfrSapProvider<C>::GetAvailableDlRbg ()
{
  return m_owner->DoGetAvailableDlRbg ();
}

template <class C>
bool
cv2x_MemberLteFfrSapProvider<C>::IsDlRbgAvailableForUe (int i, uint16_t rnti)
{
  return m_owner->DoIsDlRbgAvailableForUe (i,rnti);
}

template <class C>
std::vector <bool>
cv2x_MemberLteFfrSapProvider<C>::GetAvailableUlRbg ()
{
  return m_owner->DoGetAvailableUlRbg ();
}

template <class C>
bool
cv2x_MemberLteFfrSapProvider<C>::IsUlRbgAvailableForUe (int i, uint16_t rnti)
{
  return m_owner->DoIsUlRbgAvailableForUe (i,rnti);
}

template <class C>
void
cv2x_MemberLteFfrSapProvider<C>::ReportDlCqiInfo (const struct cv2x_FfMacSchedSapProvider::SchedDlCqiInfoReqParameters& params)
{
  m_owner->DoReportDlCqiInfo (params);
}

template <class C>
void
cv2x_MemberLteFfrSapProvider<C>::ReportUlCqiInfo (const struct cv2x_FfMacSchedSapProvider::SchedUlCqiInfoReqParameters& params)
{
  m_owner->DoReportUlCqiInfo (params);
}

template <class C>
void
cv2x_MemberLteFfrSapProvider<C>::ReportUlCqiInfo ( std::map <uint16_t, std::vector <double> > ulCqiMap )
{
  m_owner->DoReportUlCqiInfo (ulCqiMap);
}

template <class C>
uint8_t
cv2x_MemberLteFfrSapProvider<C>::GetTpc ( uint16_t rnti )
{
  return m_owner->DoGetTpc (rnti);
}

template <class C>
uint8_t
cv2x_MemberLteFfrSapProvider<C>::GetMinContinuousUlBandwidth ()
{
  return m_owner->DoGetMinContinuousUlBandwidth ();
}

/**
 * \brief Template for the implementation of the cv2x_LteFfrSapUser
 *        as a member of an owner class of type C to which all methods are
 *        forwarded.
 */
template <class C>
class cv2x_MemberLteFfrSapUser : public cv2x_LteFfrSapUser
{
public:
  /**
   * Constructor
   *
   * \param owner the owner class
   */
  cv2x_MemberLteFfrSapUser (C* owner);

  // inherited from cv2x_LteFfrSapUser
private:
  cv2x_MemberLteFfrSapUser ();
  C* m_owner; ///< the owner class

}; // end of class cv2x_LteFfrSapUser

template <class C>
cv2x_MemberLteFfrSapUser<C>::cv2x_MemberLteFfrSapUser (C* owner)
  : m_owner (owner)
{
}

} // end of namespace ns3


#endif /* CV2X_LTE_FFR_SAP_H */
