/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Base class for bar code detection.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/


// #ifdef VISP_HAVE_ZBAR //TODO: check for  aruco

#include "vpDetectorArucoCode.h"

#include <sstream>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>

#include <opencv2/core/core.hpp>

/*!
   Default constructor.
 */
vpDetectorArucoCode::vpDetectorArucoCode() : m_detector(), m_camparams()
{
  // configure the reader

  //TODO initialize camparams??
  m_detector.setThresholdParams(7, 7);
//  m_detector.setThresholdParamRange(2, 0);
}

/*!
  Detect QR codes in the image. Return true if a code is detected, false otherwise.
  \param I : Input image.
 */
bool vpDetectorArucoCode::detect(const vpImage<unsigned char> &I)
{
  bool detected = false;

  m_message.clear();
  m_polygon.clear();
  m_nb_objects = 0;

  cv::Mat opencv_image;
  vpImageConvert::convert(I, opencv_image);

  bool invert_image = true;
  if (invert_image) {
    cv::Mat sub_mat = cv::Mat::ones(opencv_image.size(), opencv_image.type())*255;
 
    //subtract the original matrix by sub_mat to give the negative output new_image
    cv::subtract(sub_mat, opencv_image, opencv_image);
  }

  float marker_size = -1; // TODO: check if marker_size is needed.

  // scan the image for barcodes
  std::vector<aruco::Marker> detected_markers=m_detector.detect(opencv_image, m_camparams, marker_size);
  
  // store number of objects
  m_nb_objects = detected_markers.size();
    
  detected = (m_nb_objects > 0);

  // extract results
  for (aruco::Marker &marker: detected_markers) {
      std::ostringstream message;
      message << marker.id;
      m_message.push_back( message.str() );
      
      std::vector<vpImagePoint> polygon;
      // aruco corners are clockwise, starting upper-left. visp_auto_tracker wants counterclockwise and starting lower-left, so we must reorder the corners. just iterate through the corners in reverse order
      for (size_t i = marker.size(); i-- > 0;) { //weird way to iterate in reverse order using a unsigned type :D
        polygon.push_back(vpImagePoint(marker[i].y, marker[i].x));
      }
      m_polygon.push_back(polygon);
  }

  return detected;
}
//#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpDetectorArucoCode.cpp.o) has no symbols
void dummy_vpDetectorArucoCode() {};
//#endif
