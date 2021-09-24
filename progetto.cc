#include <vector>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/internet-module.h"
#include "ns3/dsdv-helper.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/point-to-point-module.h"
#include "ns3/gnuplot.h"

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("DisasterRecoveryScenario");
uint16_t port = 9;

class DisasterRecoveryScenario
 {
   private:
    uint32_t m_nWifis; 
    uint32_t m_nAPs;  
    double m_totalTime; 
    std::string m_rate; 
    std::string m_phyMode; 
    uint32_t m_wifiSpeed; 
    uint32_t m_periodicUpdateInterval; 
    uint32_t m_settlingTime; 
    double m_dataStart; 
    Gnuplot2dDataset m_dataset;
    vector<pair<double,double>> plotCoordinates;
    vector<int> reachable_APs;
    vector<pair<int, int>> apCoordinates;
  
   NodeContainer nodes; 
   NodeContainer apNodes;
   NetDeviceContainer devices; 
   Ipv4InterfaceContainer interfaces; 

  public:
    DisasterRecoveryScenario ();
    void Run (uint32_t nWifis,
                 uint32_t nAPs,
                 double totalTime,
                 std::string rate,
                 std::string phyMode,
                 uint32_t nodeSpeed,
                 uint32_t periodicUpdateInterval,
                 uint32_t settlingTime,
                 double dataStart);

  private:
   void CreateNodes ();
   void CreateDevices ();
   void InstallInternetStack ();
   void InstallApplications ();
   void SetupMobility ();
   void ReceivePacket (Ptr <Socket> socket);
   Ptr <Socket> SetupPacketReceive (Ipv4Address addr, Ptr <Node> node );
 };
  
 DisasterRecoveryScenario::DisasterRecoveryScenario ()
   : m_nAPs (0)
 {}

 void
 DisasterRecoveryScenario::CreateNodes ()
 {
   std::cout << "Creating " << (unsigned) m_nWifis << " mobile wifi nodes...\n";
   nodes.Create (m_nWifis);

  std::cout << "Creating " << (unsigned) m_nAPs << " APs...\n";  
  for (uint32_t i = 0; i <= m_nAPs - 1; i++ ) {
     apNodes.Add(nodes.Get (i));
  }
  apNodes.Create(m_nAPs);
  
  NS_ASSERT_MSG (m_nWifis > m_nAPs, "Sinks must be less or equal to the number of nodes in network");
 }

void
 DisasterRecoveryScenario::SetupMobility ()
 {
   MobilityHelper mobility;
   ObjectFactory pos;
   pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
   pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=5000.0]"));
   pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=5000.0]"));
  
   std::ostringstream speedConstantRandomVariableStream;
   speedConstantRandomVariableStream << "ns3::ConstantRandomVariable[Constant="
                                     << m_wifiSpeed
                                     << "]";
  
   Ptr <PositionAllocator> taPositionAlloc = pos.Create ()->GetObject <PositionAllocator> ();
   mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel", "Speed", StringValue (speedConstantRandomVariableStream.str ()),
                              "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=2.0]"), "PositionAllocator", PointerValue (taPositionAlloc));
   mobility.SetPositionAllocator (taPositionAlloc);
   mobility.Install(nodes);

   apCoordinates = { 
      {2000, 0}, {2000, 5000}, {0, 3000}, {5000, 3000},
      {0, 0}, {0, 5000}, {5000, 5000}, {5000, 0},
      {3000, 5000}, {3000, 0}, {0, 2000}, {5000, 2000},
      {1000, 5000}, {1000, 0}, {0, 1000}, {5000, 1000},
      {4000, 5000}, {4000, 0}, {0, 4000}, {5000, 4000}
     };

   Ptr <ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

    for (uint32_t i = 0; i <= m_nWifis - 1; i++ )
     {
        positionAlloc->Add (Vector ( apCoordinates[i].first,  apCoordinates[i].second, 0.0));
     }

   mobility.SetPositionAllocator (positionAlloc);
   mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
   mobility.Install (apNodes);
 }

  void
 DisasterRecoveryScenario::CreateDevices ()
 {
  PointToPointHelper pointToPoint;
  pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  pointToPoint.SetChannelAttribute ("Delay", StringValue ("2ms"));
  NetDeviceContainer p2pDevices;

   WifiMacHelper wifiMac;
   wifiMac.SetType ("ns3::AdhocWifiMac");
   YansWifiPhyHelper wifiPhy;
   YansWifiChannelHelper wifiChannel;
   wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
   wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
   wifiPhy.SetChannel (wifiChannel.Create ());
   WifiHelper wifi;
   wifi.SetStandard (WIFI_STANDARD_80211b);
   wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue (m_phyMode), "ControlMode",
                                 StringValue (m_phyMode));
   devices = wifi.Install (wifiPhy, wifiMac, nodes);
 }
  
 void
 DisasterRecoveryScenario::InstallInternetStack ()
 {
   DsdvHelper dsdv;
   dsdv.Set ("PeriodicUpdateInterval", TimeValue (Seconds (m_periodicUpdateInterval)));
   dsdv.Set ("SettlingTime", TimeValue (Seconds (m_settlingTime)));
   InternetStackHelper stack;
   stack.SetRoutingHelper (dsdv); // has effect on the next Install ()
   stack.Install (nodes);
   Ipv4AddressHelper address;
   address.SetBase ("10.1.1.0", "255.255.255.0");
   interfaces = address.Assign (devices);
 }
  
 void
 DisasterRecoveryScenario::InstallApplications ()
 {
   for (uint32_t i = 0; i <= m_nAPs - 1; i++ )
     {
       Ptr<Node> node = NodeList::GetNode (i);
       Ipv4Address nodeAddress = node->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();
       Ptr<Socket> AP = SetupPacketReceive (nodeAddress, node);
     }
  
   for (uint32_t clientNode = 0; clientNode <= m_nWifis - 1; clientNode++ )
     {
       for (uint32_t j = 0; j <= m_nAPs - 1; j++ )
         {
           OnOffHelper onoff1 ("ns3::UdpSocketFactory", Address (InetSocketAddress (interfaces.GetAddress (j), port)));
           onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
           onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
  
           if (j != clientNode)
             {
               ApplicationContainer apps1 = onoff1.Install (nodes.Get (clientNode));
               Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
               apps1.Start (Seconds (var->GetValue (m_dataStart, m_dataStart + 1)));
               apps1.Stop (Seconds (m_totalTime));
             }
         }
     }
 }

 void
 DisasterRecoveryScenario::ReceivePacket (Ptr <Socket> socket)
 {
   Ptr <Packet> packet;
   while ((packet = socket->Recv ()))
     {
       int apID = socket->GetNode()->GetId();
       float now = Simulator::Now().GetSeconds ();

       NS_LOG_UNCOND ("At " + to_string(now) + "s " + "AP with id " + to_string(apID) + " has received a packet from the internal wifi network");

       if (std::find(reachable_APs.begin(), reachable_APs.end(), apID) == reachable_APs.end()) {
          reachable_APs.push_back(apID);
       }     

       plotCoordinates.push_back(std::make_pair(apID, now));
     }
 }
    
 Ptr <Socket>
 DisasterRecoveryScenario::SetupPacketReceive (Ipv4Address addr, Ptr <Node> node)
 {
  
   TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
   Ptr <Socket> AP = Socket::CreateSocket (node, tid);
   InetSocketAddress local = InetSocketAddress (addr, port);
   AP->Bind (local);
   AP->SetRecvCallback (MakeCallback ( &DisasterRecoveryScenario::ReceivePacket, this));
  
   return AP;
 }

 void
 DisasterRecoveryScenario::Run (uint32_t nWifis, uint32_t nAPs, double totalTime, std::string rate,
                            std::string phyMode, uint32_t nodeSpeed, uint32_t periodicUpdateInterval, uint32_t settlingTime,
                            double dataStart)
 {
   m_nWifis = nWifis;
   m_nAPs = nAPs;
   m_totalTime = totalTime;
   m_rate = rate;
   m_phyMode = phyMode;
   m_wifiSpeed = nodeSpeed;
   m_periodicUpdateInterval = periodicUpdateInterval;
   m_settlingTime = settlingTime;
   m_dataStart = dataStart;

   std::string fileNameWithNoExtension = "disaster-recovery-simulation";
   std::string graphicsFileName        = fileNameWithNoExtension + ".png";
   std::string plotFileName            = fileNameWithNoExtension + ".plt";
   std::string plotTitle               = "Network Coverage";
   std::string dataTitle               = "Packet Received";
  
   Gnuplot plot (graphicsFileName);
   
   plot.SetTitle (plotTitle);
   plot.SetTerminal ("png");
   plot.SetLegend ("Node Id", "Time (s)");
   plot.AppendExtra ("set xrange [0:" + to_string(m_nAPs) + "]");
   plot.AppendExtra ("set yrange [0:" + to_string(m_totalTime) + "]");
   
   m_dataset.SetTitle (dataTitle);
   m_dataset.SetStyle (Gnuplot2dDataset::DOTS);
  
   CreateNodes ();
   CreateDevices ();
   SetupMobility ();
   InstallInternetStack ();
   InstallApplications ();
  
   std::cout << "\nStarting simulation for " << m_totalTime << " s ...\n\n";

   Simulator::Stop (Seconds (m_totalTime));
   Simulator::Run ();

    if(Simulator::IsFinished()) {

     double percentage = (double )reachable_APs.size() / (double) m_nAPs;
     
     std::cout << "\nEnd of simulation\n";
     std::cout << "\nNumber of reachable APs: " << reachable_APs.size()<< " of " << m_nAPs;
     std::cout << "\nReachable AP ids: " <<"";
     
     for (uint32_t i = 0; i <= reachable_APs.size()-1 ; i++ )
     {
       std::cout <<reachable_APs[i] <<" ";
     }

     std::cout << "\nNetwork coverage: " << percentage * 100<< "%\n";

     for (auto iter : plotCoordinates) { 
     m_dataset.Add(iter.first, iter.second );  
   }
     
   plot.AddDataset (m_dataset);
   std::ofstream plotFile (plotFileName.c_str());
   plot.GenerateOutput (plotFile);
   plotFile.close ();

   }

   Simulator::Destroy ();
 }
     
 int main (int argc, char **argv)
 {
   DisasterRecoveryScenario test;
   uint32_t nWifis = 50;
   uint32_t nAPs = 10;
   double totalTime = 30.0;
   std::string rate ("8kbps");
   std::string phyMode ("DsssRate11Mbps");
   uint32_t nodeSpeed = 2;
   std::string appl = "all";
   uint32_t periodicUpdateInterval = 15;
   uint32_t settlingTime = 6;
   double dataStart = 0;
  
   CommandLine cmd (__FILE__);
   cmd.AddValue ("nWifis", "Number of wifi nodes[Default:50]", nWifis);
   cmd.AddValue ("nAPs", "Number of wifi AP nodes[Default:10]", nAPs);
   cmd.AddValue ("totalTime", "Total Simulation time[Default:60]", totalTime);
   cmd.Parse (argc, argv);
  
   SeedManager::SetSeed (12345);
  
   Config::SetDefault ("ns3::OnOffApplication::PacketSize", StringValue ("1000"));
   Config::SetDefault ("ns3::OnOffApplication::DataRate", StringValue (rate));
   Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue (phyMode));
   Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2000"));
  
   test = DisasterRecoveryScenario ();
   test.Run (nWifis, nAPs, totalTime, rate, phyMode, nodeSpeed, periodicUpdateInterval,
                 settlingTime, dataStart);
  
   return 0;
 }

 
  