#include "vpDetectorAlvarCode.h" // quick hack due to a stupid compilation error. if X and eigen are included, both define Success symbol (X as #define, eigen as an enum (that's the correct way) TODO: check correct position


#include "node.h"
#include "names.h"

//command line parameters
#include "cmd_line/cmd_line.h"


//tracking
#include "libauto_tracker/tracking.h"
#include "libauto_tracker/threading.h"
#include "libauto_tracker/events.h"

#include "visp_tracker/MovingEdgeSites.h"
#include "visp_tracker/KltPoints.h"

//visp includes
#include <visp/vpDisplayX.h>
#include <visp/vpMbEdgeKltTracker.h>
#include <visp/vpTime.h>

//detectors
#if VISP_VERSION_INT < VP_VERSION_INT(2,10,0)
#  include "detectors/datamatrix/detector.h"
#  include "detectors/qrcode/detector.h"
#else
#  include <visp/vpDetectorDataMatrixCode.h>
#  include <visp/vpDetectorQRCode.h>
#endif

#include <visp_bridge/camera.h>
#include <visp_bridge/image.h>
#include <visp_bridge/3dpose.h>

#include "libauto_tracker/tracking.h"

#include <resource_retriever/retriever.h>

#include "std_msgs/Int8.h"
#include "std_msgs/String.h"

#include <tf/transform_broadcaster.h>

namespace visp_auto_tracker{
        Node::Node() :
                        n_("~"),
                        queue_size_(1),
                        tracker_config_path_(),
                        model_description_(),
                        model_path_(),
                        model_name_(),
                        code_message_(),
                        debug_display_(false),
                        I_(),
                        image_header_(),
                        got_image_(false),
                        cam_(),
                        t_(NULL) {
                //get the tracker configuration file
                //this file contains all of the tracker's parameters, they are not passed to ros directly.
                n_.param<std::string>("tracker_config_path", tracker_config_path_, "");
                n_.param<bool>("debug_display", debug_display_, false);
                std::string model_full_path;
                n_.param<std::string>("model_path", model_path_, "");
                n_.param<std::string>("model_name", model_name_, "");
                n_.param<std::string>("code_message", code_message_, "");
                model_path_= model_path_[model_path_.length()-1]=='/'?model_path_:model_path_+std::string("/");
                model_full_path = model_path_+model_name_;
                tracker_config_path_ = model_full_path+".cfg";
                ROS_INFO("model full path=%s",model_full_path.c_str());

                //Parse command line arguments from config file (as ros param)
                cmd_.init(tracker_config_path_);
                cmd_.set_data_directory(model_path_); //force data path
                cmd_.set_pattern_name(model_name_); //force model name
                cmd_.set_show_fps(false);
                if (! code_message_.empty()) {
                  ROS_WARN_STREAM("Track only code with message: \"" << code_message_ << "\"");
                  cmd_.set_code_message(code_message_);
                }

                resource_retriever::Retriever r;
                resource_retriever::MemoryResource res;
                try {
                  res = r.get(std::string("file://")+cmd_.get_mbt_cad_file());
                }
                catch(...) {
                  ROS_ERROR_STREAM("Unable to read wrl or cao model file as resource: " << std::string("file://")+cmd_.get_mbt_cad_file());
                }

                model_description_.resize(res.size);
                for (unsigned int i=0; i < res.size; ++i)
                        model_description_[i] = res.data.get()[i];

                ROS_INFO("Model content=%s",model_description_.c_str());

                n_.setParam ("/model_description", model_description_);
        }

        void Node::waitForImage(){
            while ( ros::ok ()){
                if(got_image_) return;
                ros::spinOnce();
              }
        }

        //records last recieved image
        void Node::frameCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info){
                boost::mutex::scoped_lock(lock_);
                image_header_ = image->header;
                I_ = visp_bridge::toVispImageRGBa(*image); //make sure the image isn't worked on by locking a mutex
                cam_ = visp_bridge::toVispCameraParameters(*cam_info);

                got_image_ = true;
        }

        void Node::spin(){

                if(cmd_.should_exit()) return; //exit if needed

                vpMbTracker* tracker; //mb-tracker will be chosen according to config

                //create display
                vpDisplayX* d = NULL;
                if(debug_display_)
                  d = new vpDisplayX();

                //init detector based on user preference
#if VISP_VERSION_INT < VP_VERSION_INT(2,10,0)
                detectors::DetectorBase* detector = NULL;
                if (cmd_.get_detector_type() == CmdLine::ZBAR)
                        detector = new detectors::qrcode::Detector;
                else if(cmd_.get_detector_type() == CmdLine::DMTX)
                        detector = new detectors::datamatrix::Detector;
#else // ViSP >= 2.10.0. In that case we use the detectors from ViSP
                vpDetectorBase *detector = NULL;
                if (cmd_.get_detector_type() == CmdLine::ZBAR)
                        detector = new vpDetectorQRCode;
                else if(cmd_.get_detector_type() == CmdLine::DMTX)
                        detector = new vpDetectorDataMatrixCode;
#endif

#if 0
                //init tracker based on user preference
                if(cmd_.get_tracker_type() == CmdLine::KLT)
                        tracker = new vpMbKltTracker();
                else if(cmd_.get_tracker_type() == CmdLine::KLT_MBT)
                        tracker = new vpMbEdgeKltTracker();
                else if(cmd_.get_tracker_type() == CmdLine::MBT)
                        tracker = new vpMbEdgeTracker();
#else
                // Use the best tracker
                tracker = new vpMbEdgeKltTracker();
#endif
                tracker->setCameraParameters(cam_);
                tracker->setDisplayFeatures(true);

                //compile detectors and paramters into the automatic tracker.
                t_ = new tracking::Tracker(cmd_, detector, tracker, debug_display_);
                t_->start(); //start the state machine

                //subscribe to ros topics and prepare a publisher that will publish the pose
                message_filters::Subscriber<sensor_msgs::Image> raw_image_subscriber(n_, image_topic, queue_size_);
                message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_subscriber(n_, camera_info_topic, queue_size_);
                message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> image_info_sync(raw_image_subscriber, camera_info_subscriber, queue_size_);
                image_info_sync.registerCallback(boost::bind(&Node::frameCallback,this, _1, _2));
                ros::Publisher object_pose_publisher = n_.advertise<geometry_msgs::PoseStamped>(object_position_topic, queue_size_);
                ros::Publisher object_pose_covariance_publisher = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>(object_position_covariance_topic, queue_size_);
                ros::Publisher moving_edge_sites_publisher = n_.advertise<visp_tracker::MovingEdgeSites>(moving_edge_sites_topic, queue_size_);
                ros::Publisher klt_points_publisher = n_.advertise<visp_tracker::KltPoints>(klt_points_topic, queue_size_);
                ros::Publisher status_publisher = n_.advertise<std_msgs::Int8>(status_topic, queue_size_);
                ros::Publisher code_message_publisher = n_.advertise<std_msgs::String>(code_message_topic, queue_size_);
                tf::TransformBroadcaster tf_broadcaster;

                //wait for an image to be ready
                waitForImage();
                {
                        //when an image is ready tell the tracker to start searching for patterns
                        boost::mutex::scoped_lock(lock_);
                        if(debug_display_) {
                          d->init(I_); //also init display
                          vpDisplay::setTitle(I_, "visp_auto_tracker debug display");
                        }

                        t_->process_event(tracking::select_input(I_));
                }

                unsigned int iter=0;
                geometry_msgs::PoseStamped ps;
                geometry_msgs::PoseWithCovarianceStamped ps_cov;
                tf::Transform tr;
                
                ros::Rate rate(30); //init 25fps publishing frequency
                const int TRACK_SUCCESSFUL = 3; // this comes from the finite-state-machine of the visp_auto_tracker.
                while(ros::ok()){
                  double t = vpTime::measureTimeMs();
                        boost::mutex::scoped_lock(lock_);
                        //process the new frame with the tracker
                        t_->process_event(tracking::input_ready(I_,cam_,iter));
                        //When the tracker is tracking, it's in the tracking::TrackModel state
                        //Access this state and broadcast the pose
                        tracking::TrackModel& track_model = t_->get_state<tracking::TrackModel&>();
 
                        ps.pose = visp_bridge::toGeometryMsgsPose(track_model.cMo); //convert
                        
                        
                        std::vector<vpPoint> points3D_outer = track_model.points3D_outer;
						
						tf::Quaternion orientation_towards_camera;
                        tf::quaternionMsgToTF(ps.pose.orientation, orientation_towards_camera);
                        tf::Quaternion quaternion_rotation;
                        //quaternion_rotation.setRPY(M_PI, M_PI_2, 0);
                        quaternion_rotation.setRPY(-M_PI_2, M_PI_2, 0);
                        //quaternion_rotation.setRPY(0, 0, 0);
                        tf::Quaternion orientation_away_from_camera;
                        orientation_away_from_camera = orientation_towards_camera *  quaternion_rotation;

                        tf::quaternionTFToMsg(orientation_away_from_camera, ps.pose.orientation);

						// only publish marker poses if tracking is successful
                        if (*(t_->current_state()) == TRACK_SUCCESSFUL) { 
                            
                            // set pose
                            ps.header = image_header_;
                            ps.header.frame_id = image_header_.frame_id; // tracker_ref_frame;
                            
                            // set pose with covariance
                            ps_cov.pose.pose = ps.pose;
                            ps_cov.header = image_header_;
                            ps_cov.header.frame_id = image_header_.frame_id; //  tracker_ref_frame;

                            for (unsigned i = 0; i < track_model.covariance.getRows(); ++i)
                            {
                                for (unsigned j = 0; j < track_model.covariance.getCols(); ++j)
                                {
                                    unsigned idx = i * track_model.covariance.getCols() + j;
                                    if (idx >= 36)
                                        continue;
                                    ps_cov.pose.covariance[idx] = track_model.covariance[i][j];
                                }
                            }

                            // convert pose to tf
                            tf::poseMsgToTF(ps.pose, tr);


                            // Publish resulting pose if we have subscribers
                            if (object_pose_publisher.getNumSubscribers	() > 0)
                            {
                                object_pose_publisher.publish(ps);
                            }

                            // Publish resulting pose covariance matrix if we have subscribers
                            if (object_pose_covariance_publisher.getNumSubscribers	() > 0)
                            {
                                object_pose_covariance_publisher.publish(ps_cov);
                            }
                            
                            // Publish to TF only if tracking is stable, i.e. the max covariance is below some threshold
                            double cov_max_acceptable = 1e-3;
                            double cov_max = *std::max_element(ps_cov.pose.covariance.begin(), ps_cov.pose.covariance.end()); // XXX: maybe it would be better to check the covariance directly from the model, not from the msg, but now it is easire (ie do not have to check a new api)
                            if (cov_max < cov_max_acceptable) {
                                std::string marker_frame = detector->getMessage( cmd_.get_code_message_index() ); //marker_frame_prefix + detector->getMessage( cmd_.get_code_message_index() );
                                tf_broadcaster.sendTransform(tf::StampedTransform(tr, image_header_.stamp, image_header_.frame_id /* tracker_ref_frame*/, marker_frame));
                            }
                            else {
                                ROS_INFO_THROTTLE(1, "Bad covariance!! %f", cov_max);
                            }
                        }

                        // Publish state machine status.
                        if (status_publisher.getNumSubscribers	() > 0)
                        {
                            std_msgs::Int8 status;
                            status.data = (unsigned char)(*(t_->current_state()));
                            status_publisher.publish(status);
                        }

                        // Publish moving edge sites.
                        if (moving_edge_sites_publisher.getNumSubscribers	() > 0)
                        {
                            visp_tracker::MovingEdgeSitesPtr sites (new visp_tracker::MovingEdgeSites);
                            // Test if we are in the state tracking::TrackModel. In that case the pose is good;
                            // we can send the moving edges. Otherwise we send an empty list of features
                            if (*(t_->current_state()) == 3) {
                              t_->updateMovingEdgeSites(sites);
                            }

                            sites->header = image_header_;
                            moving_edge_sites_publisher.publish(sites);
                        }

                        // Publish KLT points.
                        if (klt_points_publisher.getNumSubscribers	() > 0)
                        {
                            visp_tracker::KltPointsPtr klt (new visp_tracker::KltPoints);
                            // Test if we are in the state tracking::TrackModel. In that case the pose is good;
                            // we can send the klt points. Otherwise we send an empty list of features
                            if (*(t_->current_state()) == 3) {
                              t_->updateKltPoints(klt);
                            }
                            klt->header = image_header_;
                            klt_points_publisher.publish(klt);
                        }

                        if (code_message_publisher.getNumSubscribers() > 0)
                        {
                          std_msgs::String message;
                          if (*(t_->current_state()) == 3) { // Tracking successful
#if VISP_VERSION_INT < VP_VERSION_INT(2,10,0)
                            message.data = detector->get_message();
#else
                            message.data = detector->getMessage( cmd_.get_code_message_index() );
#endif
                          }
                          else {
                            message.data = std::string();
                          }
                          code_message_publisher.publish(message);
                          ROS_INFO_STREAM("Code with message \"" <<  message.data << "\" under tracking");
                        }

                        ros::spinOnce();
                        rate.sleep();
                        if (cmd_.show_fps())
                          std::cout << "Tracking done in " << vpTime::measureTimeMs() - t << " ms" << std::endl;
                }
                t_->process_event(tracking::finished());
        }
}
