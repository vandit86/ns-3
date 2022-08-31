#ifndef SIMPLECAMSENDER_H
#define SIMPLECAMSENDER_H

#include "ns3/traci-client.h"
#include "ns3/application.h"
#include "ns3/asn_utils.h"

#include "ns3/denBasicService.h"
#include "ns3/caBasicService.h"

#include "ns3/trace-source-accessor.h"
#include "ns3/callback.h"
#include "ns3/traced-callback.h"




namespace ns3 {

class simpleCAMSender : public Application
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  simpleCAMSender ();

  virtual ~simpleCAMSender ();

  // void receiveCAM(CAM_t *cam, Address address);
  void receiveCAM(asn1cpp::Seq<CAM> cam, Address address);

  void receiveDENM(denData denm, Address from);

  void StopApplicationNow ();

protected:
  virtual void DoDispose (void);

private:

  DENBasicService m_denService; //!< DEN Basic Service object
  CABasicService m_caService; //!< CA Basic Service object
  Ptr<btp> m_btp; //! BTP object
  Ptr<GeoNet> m_geoNet; //! GeoNetworking Object

  Ptr<Socket> m_socket;

  virtual void StartApplication (void);
  virtual void StopApplication (void);

  Ptr<TraciClient> m_client; //!< TraCI SUMO client

  std::string m_id;                 //!< vehicle id
  //StationID_t m_s_id = 1001;      // station id for cam filtering (hardcoded)
  bool m_real_time;                 //!< To decide wheter to use realtime scheduler
  bool m_is_rsu = false;            // is this node is stationary RSU 
  uint32_t m_iface = 0;             // interface used when send CAM (default 0)
  uint32_t m_rsu_cam_interval_ms;   // rsu cam generation interval in ms

  //EventId m_sendCamEvent; //!< Event to send the CAM
  
  //!< CAM callback (station id, latitude, longitude); 
  //Callback<void, long, double, double> m_cam_callback;  

  /* Counters */
  int m_cam_sent;
};

} // namespace ns3

#endif /* SIMPLECAMSENDER_H */

