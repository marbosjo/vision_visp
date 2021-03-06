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

#ifndef __vpDetectorAlvarCode_h__
#define __vpDetectorAlvarCode_h__

#include <vector>
#include <utility>
#include <string>

// TODO check if it is needed to the "Success error"
//#if defined(Success)
//#undef Success
//#endif
#include <ar_track_alvar/MarkerDetector.h>

#include <visp3/core/vpConfig.h>

//#ifdef VISP_HAVE_ZBAR //TODO: change for aruco


#include <visp3/detection/vpDetectorBase.h>
#include <visp3/core/vpImage.h>



/*!
  \class vpDetectorAlvarCode
  \ingroup group_detection_barcode
  Base class for bar code detector. This class is a wrapper over libzbar
  available from http://zbar.sourceforge.net/
  The detect() function allows to detect multiple QR codes in an image. Once detected,
  for each QR code it is possible to retrieve the location of the corners using getPolygon(),
  the encoded message using getMessage(), the bounding box using getBBox() and the center
  of gravity using getCog().
  The following sample code shows how to use this class to detect QR codes in an image.
  \code
#include <visp3/detection/vpDetectorAlvarCode.h>
#include <visp3/io/vpImageIo.h>
int main()
{
#ifdef VISP_HAVE_ZBAR
  vpImage<unsigned char> I;
  vpImageIo::read(I, "bar-code.pgm");
  vpDetectorAlvarCode detector;
  bool status = detector.detect(I);
  if (status) {
    for(size_t i=0; i < detector.getNbObjects(); i++) {
      std::cout << "Bar code " << i << ":" << std::endl;
      std::vector<vpImagePoint> p = detector.getPolygon(i);
      for(size_t j=0; j < p.size(); j++)
        std::cout << "  Point " << j << ": " << p[j] << std::endl;
      std::cout << "  Message: \"" << detector.getMessage(i) << "\"" << std::endl;
    }
  }
#endif
}
  \endcode
  The previous example may produce results like:
  \code
Bar code 0:
  Point 0: 48, 212
  Point 1: 57, 84
  Point 2: 188, 92
  Point 3: 183, 220
  Message: "qrcode 2"
Bar code 1:
  Point 0: 26, 550
  Point 1: 35, 409
  Point 2: 174, 414
  Point 3: 167, 555
  Message: "qrcode 1"
  \endcode
  Other examples are also provided in tutorial-barcode-detector.cpp and
  tutorial-barcode-detector-live.cpp
 */
class VISP_EXPORT vpDetectorAlvarCode : public vpDetectorBase
{
protected:
  alvar::MarkerDetector<alvar::MarkerData> m_detector;
  alvar::Camera m_camparams;
  double m_minmarkersizeinpixels;
public:
  vpDetectorAlvarCode();
  virtual ~vpDetectorAlvarCode() {};
  bool detect(const vpImage<unsigned char> &I);
};

#endif
//#endif //TODO: reset when proper aruco detection is carried out
