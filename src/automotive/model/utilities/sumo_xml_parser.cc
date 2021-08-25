/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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

 * Created by:
 *  Marco Malinverno, Politecnico di Torino (marco.malinverno1@gmail.com)
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
*/

#include "sumo_xml_parser.h"

namespace ns3
{
  int XML_rou_count_vehicles(xmlDocPtr doc)
  {
      xmlXPathContextPtr xpathCtx;
      xmlXPathObjectPtr xpathObj;

      int num_vehicles=-1;

      // Create xPath to select all the 'vehicle' nodes in the rou.xml file
      xpathCtx = xmlXPathNewContext(doc);
      if(xpathCtx == NULL) {
          return -1;
      }

      // Evaluate the xPath expression "//vehicle" to look for all the "<vehicle>" elements
      xpathObj = xmlXPathEvalExpression((xmlChar *)"//vehicle",xpathCtx);
      if(xpathObj == NULL || xpathObj->nodesetval==NULL) {
          xmlXPathFreeContext(xpathCtx);
          return -1;
      }

      num_vehicles = xpathObj->nodesetval->nodeNr;

      xmlXPathFreeObject(xpathObj);
      xmlXPathFreeContext(xpathCtx);

      return num_vehicles;
  }
}
