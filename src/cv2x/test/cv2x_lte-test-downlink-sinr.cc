/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 * Author: Manuel Requena <manuel.requena@cttc.es>
 * Modified by Marco Miozzo <mmiozzo@ctt.es>
 *     Extend to Data and Ctrl frames
 */

#include "ns3/simulator.h"

#include "ns3/log.h"

#include "ns3/spectrum-test.h"
#include "ns3/boolean.h"
#include "ns3/cv2x_lte-phy-tag.h"
#include "cv2x_lte-test-ue-phy.h"
#include "ns3/cv2x_lte-spectrum-signal-parameters.h"

#include "cv2x_lte-test-downlink-sinr.h"
#include <ns3/cv2x_lte-control-messages.h>
#include "ns3/cv2x_lte-helper.h"
#include <ns3/cv2x_lte-chunk-processor.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("cv2x_LteDownlinkSinrTest");

/**
 * Test 1.1 SINR calculation in downlink
 */

/**
 * TestSuite
 */

cv2x_LteDownlinkSinrTestSuite::cv2x_LteDownlinkSinrTestSuite ()
  : TestSuite ("lte-downlink-sinr", SYSTEM)
{
  /**
   * Build Spectrum Model values for the TX signal
   */
  Ptr<SpectrumModel> sm;

  Bands bands;
  BandInfo bi;

  bi.fl = 2.400e9;
  bi.fc = 2.410e9;
  bi.fh = 2.420e9;
  bands.push_back (bi);

  bi.fl = 2.420e9;
  bi.fc = 2.431e9;
  bi.fh = 2.442e9;
  bands.push_back (bi);

  sm = Create<SpectrumModel> (bands);

  /**
   * TX signal #1: Power Spectral Density (W/Hz) of the signal of interest = [-46 -48] dBm and BW = [20 22] MHz
   */
  Ptr<SpectrumValue> rxPsd1 = Create<SpectrumValue> (sm);
  (*rxPsd1)[0] = 1.255943215755e-15;
  (*rxPsd1)[1] = 7.204059965732e-16;

  Ptr<SpectrumValue> theoreticalSinr1 = Create<SpectrumValue> (sm);
  (*theoreticalSinr1)[0] = 3.72589167251055;
  (*theoreticalSinr1)[1] = 3.72255684126076;

  AddTestCase (new cv2x_LteDownlinkDataSinrTestCase (rxPsd1, theoreticalSinr1, "sdBm = [-46 -48]"), TestCase::QUICK);
  AddTestCase (new cv2x_LteDownlinkCtrlSinrTestCase (rxPsd1, theoreticalSinr1, "sdBm = [-46 -48]"), TestCase::QUICK);

  /**
   * TX signal #2: Power Spectral Density (W/Hz) of the signal of interest = [-63 -61] dBm and BW = [20 22] MHz
   */
  Ptr<SpectrumValue> rxPsd2 = Create<SpectrumValue> (sm);
  (*rxPsd2)[0] = 2.505936168136e-17;
  (*rxPsd2)[1] = 3.610582885110e-17;

  Ptr<SpectrumValue> theoreticalSinr2 = Create<SpectrumValue> (sm);
  (*theoreticalSinr2)[0] = 0.0743413124381667;
  (*theoreticalSinr2)[1] = 0.1865697965291756;

  AddTestCase (new cv2x_LteDownlinkDataSinrTestCase (rxPsd2, theoreticalSinr2, "sdBm = [-63 -61]"), TestCase::QUICK);
  AddTestCase (new cv2x_LteDownlinkCtrlSinrTestCase (rxPsd2, theoreticalSinr2, "sdBm = [-63 -61]"), TestCase::QUICK);
  

}

static cv2x_LteDownlinkSinrTestSuite lteDownlinkSinrTestSuite;


/**
 * TestCase Data
 */

cv2x_LteDownlinkDataSinrTestCase::cv2x_LteDownlinkDataSinrTestCase (Ptr<SpectrumValue> sv, Ptr<SpectrumValue> sinr, std::string name)
  : TestCase ("SINR calculation in downlink Data frame: " + name),
    m_sv (sv),
    m_sm (sv->GetSpectrumModel ()),
    m_expectedSinr (sinr)
{
  NS_LOG_INFO ("Creating LenaDownlinkSinrTestCase");
}

cv2x_LteDownlinkDataSinrTestCase::~cv2x_LteDownlinkDataSinrTestCase ()
{
}


void
cv2x_LteDownlinkDataSinrTestCase::DoRun (void)
{
  Config::SetDefault ("ns3::cv2x_LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue (false));
  /**
   * Instantiate a single receiving cv2x_LteSpectrumPhy
   */
  Ptr<cv2x_LteSpectrumPhy> dlPhy = CreateObject<cv2x_LteSpectrumPhy> ();
  Ptr<cv2x_LteSpectrumPhy> ulPhy = CreateObject<cv2x_LteSpectrumPhy> ();
  Ptr<cv2x_LteTestUePhy> uePhy = CreateObject<cv2x_LteTestUePhy> (dlPhy, ulPhy);
  uint16_t cellId = 100;
  dlPhy->SetCellId (cellId);
  ulPhy->SetCellId (cellId);

  Ptr<cv2x_LteChunkProcessor> chunkProcessor = Create<cv2x_LteChunkProcessor> ();
  cv2x_LteSpectrumValueCatcher actualSinrCatcher;
  chunkProcessor->AddCallback (MakeCallback (&cv2x_LteSpectrumValueCatcher::ReportValue, &actualSinrCatcher));
  dlPhy->AddDataSinrChunkProcessor (chunkProcessor);

  /**
   * Generate several calls to cv2x_LteSpectrumPhy::StartRx corresponding to 
   * several signals. One will be the signal of interest, i.e., the
   *  cv2x_LteSpectrumSignalParametersDataFrame of the Packet burst
   * will have the same CellId of the receiving PHY; the others will have 
   * a different CellId and hence will be the interfering signals
   */

  // Number of packet bursts (1 data + 4 interferences)
  const int numOfPbs = 5;

  // Number of packets in the packet bursts
  const int numOfPkts = 10;

  // Packet bursts
  Ptr<PacketBurst> packetBursts[numOfPbs];

  // Packets
  Ptr<Packet> pkt[numOfPbs][numOfPkts];
  
  // Packet bursts cellId
  uint16_t pbCellId[numOfPbs];

  /**
   * Build packet burst
   */
  for ( int pb = 0 ; pb < numOfPbs ; pb++ )
    {
      // Create packet burst
      packetBursts[pb] = CreateObject<PacketBurst> ();
      pbCellId[pb] = cellId * (pb + 1);

      // Create packets and add them to the burst
      for ( int i = 0 ; i < numOfPkts ; i++ )
        {
          pkt[pb][i] = Create<Packet> (1000);

          packetBursts[pb]->AddPacket ( pkt[pb][i] );
        }
    }


  Ptr<SpectrumValue> noisePsd = Create<SpectrumValue> (m_sm);
  Ptr<SpectrumValue> i1 = Create<SpectrumValue> (m_sm);
  Ptr<SpectrumValue> i2 = Create<SpectrumValue> (m_sm);
  Ptr<SpectrumValue> i3 = Create<SpectrumValue> (m_sm);
  Ptr<SpectrumValue> i4 = Create<SpectrumValue> (m_sm);

  (*noisePsd)[0] = 5.000000000000e-19;
  (*noisePsd)[1] = 4.545454545455e-19;

  (*i1)[0] = 5.000000000000e-18;
  (*i2)[0] = 5.000000000000e-16;
  (*i3)[0] = 1.581138830084e-16;
  (*i4)[0] = 7.924465962306e-17;
  (*i1)[1] = 1.437398936440e-18;
  (*i2)[1] = 5.722388235428e-16;
  (*i3)[1] = 7.204059965732e-17;
  (*i4)[1] = 5.722388235428e-17;

  Time ts  = Seconds (1);
  Time ds  = Seconds (1);
  Time ti1 = Seconds (0);
  Time di1 = Seconds (3);
  Time ti2 = Seconds (0.7);
  Time di2 = Seconds (1);
  Time ti3 = Seconds (1.2);
  Time di3 = Seconds (1);
  Time ti4 = Seconds (1.5);
  Time di4 = Seconds (0.1);

  dlPhy->SetNoisePowerSpectralDensity (noisePsd);

  /**
   * Schedule the reception of the data signal plus the interference signals
   */

  // eNB sends data to 2 UEs through 2 subcarriers
  Ptr<cv2x_LteSpectrumSignalParametersDataFrame> sp1 = Create<cv2x_LteSpectrumSignalParametersDataFrame> ();
  sp1->psd = m_sv;
  sp1->txPhy = 0;
  sp1->duration = ds;
  sp1->packetBurst = packetBursts[0];
  sp1->cellId = pbCellId[0];
  Simulator::Schedule (ts, &cv2x_LteSpectrumPhy::StartRx, dlPhy, sp1);


  Ptr<cv2x_LteSpectrumSignalParametersDataFrame> ip1 = Create<cv2x_LteSpectrumSignalParametersDataFrame> ();
  ip1->psd = i1;
  ip1->txPhy = 0;
  ip1->duration = di1;
  ip1->packetBurst = packetBursts[1];
  ip1->cellId = pbCellId[1];
  Simulator::Schedule (ti1, &cv2x_LteSpectrumPhy::StartRx, dlPhy, ip1);

  Ptr<cv2x_LteSpectrumSignalParametersDataFrame> ip2 = Create<cv2x_LteSpectrumSignalParametersDataFrame> ();
  ip2->psd = i2;
  ip2->txPhy = 0;
  ip2->duration = di2;
  ip2->packetBurst = packetBursts[2];
  ip2->cellId = pbCellId[2];
  Simulator::Schedule (ti2, &cv2x_LteSpectrumPhy::StartRx, dlPhy, ip2);

  Ptr<cv2x_LteSpectrumSignalParametersDataFrame> ip3 = Create<cv2x_LteSpectrumSignalParametersDataFrame> ();
  ip3->psd = i3;
  ip3->txPhy = 0;
  ip3->duration = di3;
  ip3->packetBurst = packetBursts[3];
  ip3->cellId = pbCellId[3];
  Simulator::Schedule (ti3, &cv2x_LteSpectrumPhy::StartRx, dlPhy, ip3);

  Ptr<cv2x_LteSpectrumSignalParametersDataFrame> ip4 = Create<cv2x_LteSpectrumSignalParametersDataFrame> ();
  ip4->psd = i4;
  ip4->txPhy = 0;
  ip4->duration = di4;
  ip4->packetBurst = packetBursts[4];
  ip4->cellId = pbCellId[4];
  Simulator::Schedule (ti4, &cv2x_LteSpectrumPhy::StartRx, dlPhy, ip4);

  Simulator::Stop (Seconds (5.0));
  Simulator::Run ();

  NS_LOG_INFO ("Data Frame - Theoretical SINR: " << *m_expectedSinr);
  NS_LOG_INFO ("Data Frame - Calculated SINR: " << *(actualSinrCatcher.GetValue ()));
 
  NS_TEST_ASSERT_MSG_SPECTRUM_VALUE_EQ_TOL(*(actualSinrCatcher.GetValue ()), *m_expectedSinr, 0.0000001, "Data Frame - Wrong SINR !");
  dlPhy->Dispose ();
  Simulator::Destroy ();
}



/**
* TestCase CTRL
*/

cv2x_LteDownlinkCtrlSinrTestCase::cv2x_LteDownlinkCtrlSinrTestCase (Ptr<SpectrumValue> sv, Ptr<SpectrumValue> sinr, std::string name)
: TestCase ("SINR calculation in downlink Ctrl Frame: " + name),
m_sv (sv),
m_sm (sv->GetSpectrumModel ()),
m_expectedSinr (sinr)
{
  NS_LOG_INFO ("Creating LenaDownlinkCtrlSinrTestCase");
}

cv2x_LteDownlinkCtrlSinrTestCase::~cv2x_LteDownlinkCtrlSinrTestCase ()
{
}

void
cv2x_LteDownlinkCtrlSinrTestCase::DoRun (void)
{
  /**
  * Instantiate a single receiving cv2x_LteSpectrumPhy
  */
  Config::SetDefault ("ns3::cv2x_LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue (false));
  Ptr<cv2x_LteSpectrumPhy> dlPhy = CreateObject<cv2x_LteSpectrumPhy> ();
  Ptr<cv2x_LteSpectrumPhy> ulPhy = CreateObject<cv2x_LteSpectrumPhy> ();
  Ptr<cv2x_LteTestUePhy> uePhy = CreateObject<cv2x_LteTestUePhy> (dlPhy, ulPhy);
  uint16_t cellId = 100;
  dlPhy->SetCellId (cellId);
  ulPhy->SetCellId (cellId);
  
  Ptr<cv2x_LteChunkProcessor> chunkProcessor = Create<cv2x_LteChunkProcessor> ();
  cv2x_LteSpectrumValueCatcher actualSinrCatcher;
  chunkProcessor->AddCallback (MakeCallback (&cv2x_LteSpectrumValueCatcher::ReportValue, &actualSinrCatcher));
  dlPhy->AddCtrlSinrChunkProcessor (chunkProcessor);

  /**
  * Generate several calls to cv2x_LteSpectrumPhy::StartRx corresponding to several signals. One will be the signal of interest, i.e., the
  *  cv2x_LteSpectrumSignalParametersDlCtrlFrame of the first signal will have the
  *  same CellId of the receiving PHY; the others will have a different 
  *  CellId and hence will be the interfering signals
  */
  
  // Number of ctrl bursts (1 data + 4 interferences)
  const int numOfUes = 5;
  
  // Number of control messages in the list
  const int numOfCtrlMsgs = 10;
  
  // control messages in the list
  std::list<Ptr<cv2x_LteControlMessage> > ctrlMsgList[numOfUes];
  
  // signals cellId
  uint16_t pbCellId[numOfUes];
  
  /**
  * Build ctrl msg lists
  */
  for ( int pb = 0 ; pb < numOfUes ; pb++ )
  {
    pbCellId[pb] = cellId * (pb + 1);
    
    // Create ctrl msg and add them to the list
    for ( int i = 0 ; i < numOfCtrlMsgs ; i++ )
      {
        Ptr<cv2x_DlDciLteControlMessage> msg = Create<cv2x_DlDciLteControlMessage> ();
        cv2x_DlDciListElement_s dci;
        msg->SetDci (dci);
        ctrlMsgList[pb].push_back (msg);
      }
  }
  
  
  Ptr<SpectrumValue> noisePsd = Create<SpectrumValue> (m_sm);
  Ptr<SpectrumValue> i1 = Create<SpectrumValue> (m_sm);
  Ptr<SpectrumValue> i2 = Create<SpectrumValue> (m_sm);
  Ptr<SpectrumValue> i3 = Create<SpectrumValue> (m_sm);
  Ptr<SpectrumValue> i4 = Create<SpectrumValue> (m_sm);
  
  (*noisePsd)[0] = 5.000000000000e-19;
  (*noisePsd)[1] = 4.545454545455e-19;
  
  (*i1)[0] = 5.000000000000e-18;
  (*i2)[0] = 5.000000000000e-16;
  (*i3)[0] = 1.581138830084e-16;
  (*i4)[0] = 7.924465962306e-17;
  (*i1)[1] = 1.437398936440e-18;
  (*i2)[1] = 5.722388235428e-16;
  (*i3)[1] = 7.204059965732e-17;
  (*i4)[1] = 5.722388235428e-17;
  
  Time ts  = Seconds (1);
  Time ds  = Seconds (1);
  Time ti1 = Seconds (0);
  Time di1 = Seconds (3);
  Time ti2 = Seconds (0.7);
  Time di2 = Seconds (1);
  Time ti3 = Seconds (1.2);
  Time di3 = Seconds (1);
  Time ti4 = Seconds (1.5);
  Time di4 = Seconds (0.1);
  
  dlPhy->SetNoisePowerSpectralDensity (noisePsd);
  
  /**
  * Schedule the reception of the data signal plus the interference signals
  */
  
  // eNB sends data to 2 UEs through 2 subcarriers
  Ptr<cv2x_LteSpectrumSignalParametersDlCtrlFrame> sp1 = Create<cv2x_LteSpectrumSignalParametersDlCtrlFrame> ();
  sp1->psd = m_sv;
  sp1->txPhy = 0;
  sp1->duration = ds;
  sp1->ctrlMsgList = ctrlMsgList[0];
  sp1->cellId = pbCellId[0];
  sp1->pss = false;
  Simulator::Schedule (ts, &cv2x_LteSpectrumPhy::StartRx, dlPhy, sp1);
  
  
  Ptr<cv2x_LteSpectrumSignalParametersDlCtrlFrame> ip1 = Create<cv2x_LteSpectrumSignalParametersDlCtrlFrame> ();
  ip1->psd = i1;
  ip1->txPhy = 0;
  ip1->duration = di1;
  ip1->ctrlMsgList = ctrlMsgList[1];
  ip1->cellId = pbCellId[1];
  ip1->pss = false;
  Simulator::Schedule (ti1, &cv2x_LteSpectrumPhy::StartRx, dlPhy, ip1);
  
  Ptr<cv2x_LteSpectrumSignalParametersDlCtrlFrame> ip2 = Create<cv2x_LteSpectrumSignalParametersDlCtrlFrame> ();
  ip2->psd = i2;
  ip2->txPhy = 0;
  ip2->duration = di2;
  ip2->ctrlMsgList = ctrlMsgList[2];
  ip2->cellId = pbCellId[2];
  ip2->pss = false;
  Simulator::Schedule (ti2, &cv2x_LteSpectrumPhy::StartRx, dlPhy, ip2);
  
  Ptr<cv2x_LteSpectrumSignalParametersDlCtrlFrame> ip3 = Create<cv2x_LteSpectrumSignalParametersDlCtrlFrame> ();
  ip3->psd = i3;
  ip3->txPhy = 0;
  ip3->duration = di3;
  ip3->ctrlMsgList = ctrlMsgList[3];
  ip3->cellId = pbCellId[3];
  ip3->pss = false;
  Simulator::Schedule (ti3, &cv2x_LteSpectrumPhy::StartRx, dlPhy, ip3);
  
  Ptr<cv2x_LteSpectrumSignalParametersDlCtrlFrame> ip4 = Create<cv2x_LteSpectrumSignalParametersDlCtrlFrame> ();
  ip4->psd = i4;
  ip4->txPhy = 0;
  ip4->duration = di4;
  ip4->ctrlMsgList = ctrlMsgList[4];
  ip4->cellId = pbCellId[4];
  ip4->pss = false;
  Simulator::Schedule (ti4, &cv2x_LteSpectrumPhy::StartRx, dlPhy, ip4);
  
  Simulator::Stop (Seconds (5.0));
  Simulator::Run ();
  
  NS_LOG_INFO ("Ctrl Frame - Theoretical SINR: " << *m_expectedSinr);
  NS_LOG_INFO ("Ctrl Frame - Calculated SINR: " << *(actualSinrCatcher.GetValue ()));
  
  NS_TEST_ASSERT_MSG_SPECTRUM_VALUE_EQ_TOL(*(actualSinrCatcher.GetValue ()), *m_expectedSinr, 0.0000001, "Data Frame - Wrong SINR !");
  dlPhy->Dispose ();
  Simulator::Destroy ();
}
