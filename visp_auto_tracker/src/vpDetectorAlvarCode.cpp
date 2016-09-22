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

#include "vpDetectorAlvarCode.h"

#include <sstream>
#include <algorithm>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>

#include <opencv2/core/core.hpp>

/*!
  Default constructor.
 */
vpDetectorAlvarCode::vpDetectorAlvarCode() : m_detector(), m_camparams()
{
    // configure the reader

    m_detector.SetMarkerSize(10); // in cm... needed?
    m_camparams.SetSimpleCalib(1280, 720, 0.5); // TODO set proper
    m_minmarkersizeinpixels = 1000; // TODO: select proper value
}

/*!
  Detect QR codes in the image. Return true if a code is detected, false otherwise.
  \param I : Input image.
 */
bool vpDetectorAlvarCode::detect(const vpImage<unsigned char> &I)
{
    bool detected = false;

    m_message.clear();
    m_polygon.clear();
    m_nb_objects = 0;

    IplImage* ipl_image = NULL;
    vpImageConvert::convert(I, ipl_image);

    bool invert_image = true;
    if (invert_image) {
        IplImage *sub_image = cvCreateImage(cvGetSize(ipl_image),ipl_image->depth,ipl_image->nChannels);
        cvSet(sub_image, cvScalar(255));
        cvSub(sub_image, ipl_image, ipl_image);
        cvReleaseImage(&sub_image);
    }

    float marker_size = -1; // TODO: check if marker_size is needed.

    // scan the image for barcodes
    m_detector.Detect(ipl_image, &m_camparams); // TODO: more params, true, false, max_new_marker_error, max_track_error, CVSEQ, true);

    // store number of objects
    //m_nb_objects = m_detector.markers->size();

    // extract results
    for (alvar::MarkerData &marker: *m_detector.markers) {
        std::vector<vpImagePoint> polygon;
        std::vector<cv::Point> cv_polygon;
        for (alvar::PointDouble corner: marker.marker_corners_img) {
            polygon.push_back(vpImagePoint(corner.y, corner.x));
            cv_polygon.push_back(cv::Point(corner.y, corner.x));
        }
        //      std::rotate(polygon.begin(), polygon.begin()+2, polygon.end());

        double area = cv::contourArea(cv_polygon);
        if (area > m_minmarkersizeinpixels) {
            m_polygon.push_back(polygon);
            if (marker.GetId() == 7) {// id = 8 is the docking station, 7 is the new docking marker
                m_message.push_back( "docking_station_marker" ); // TODO: check numbers, names
            }
            else {
                std::ostringstream message;
                message << marker.GetId();
                m_message.push_back( message.str() );
            }
        }
    }

    m_nb_objects = m_polygon.size();
    detected = (m_nb_objects > 0);
    cvReleaseImage(&ipl_image);

    return detected;
}
//#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpDetectorAlvarCode.cpp.o) has no symbols
void dummy_vpDetectorAlvarCode() {};
//#endif
