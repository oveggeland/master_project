#ifndef IMG_EVENT_HANDLER_
#define IMG_EVENT_HANDLER_
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <pluginlib/class_list_macros.h>

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <synchronizer/GetTimeStamp.h>

#include "device_event_handler.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

class ImageEventHandler : public ImageEvent
{
	public:
		ImageEventHandler(std::string p_cam_name, CameraPtr p_cam_ptr, image_transport::CameraPublisher *p_cam_pub_ptr, 
							boost::shared_ptr<camera_info_manager::CameraInfoManager> p_c_info_mgr_ptr, 
							DeviceEventHandler* p_device_event_handler_ptr, bool p_exp_time_comp_flag)
		{
			m_cam_name = p_cam_name;
			m_cam_ptr = p_cam_ptr;
			m_cam_pub_ptr = p_cam_pub_ptr;
			m_c_info_mgr_ptr = p_c_info_mgr_ptr;
			m_device_event_handler_ptr = p_device_event_handler_ptr;
			m_exp_time_comp_flag = p_exp_time_comp_flag;
			image_msg = boost::make_shared<sensor_msgs::Image>();
			img_count = 0;
			// config_all_chunk_data();
		}
		~ImageEventHandler()
		{
			m_cam_ptr = nullptr;
		}
		void OnImageEvent(ImagePtr image)
		{
			// --------------- Get time stamp from synchronizer node! ---------- //
			const std::map<std::string, std::string> m_header {{"id", m_cam_name}};
			ros::NodeHandle n;
			ros::ServiceClient service = n.serviceClient<synchronizer::GetTimeStamp>("get_time_stamp", false, m_header);
			synchronizer::GetTimeStamp srv;
			srv.request.seq = img_count;
			service.call(srv);

			ros::Time trigger_time = ros::Time(srv.response.secs, srv.response.nsecs);
			// Wait for IMU to have posted time stamp on server
			while (trigger_time == ros::Time(0)){
				if (m_cam_ptr->TransferQueueCurrentBlockCount.GetValue() > 0){
					ROS_WARN("Too much time passed, new image waiting...");
					return;
				}
				service.call(srv);
				trigger_time = ros::Time(srv.response.secs, srv.response.nsecs);
			}
			img_count ++;
			// ----------------------------------------------------------------- //

			ros::Time image_stamp = trigger_time;
		
			if(m_exp_time_comp_flag)
			{
				// get the exposure time
				double exp_time = double(m_cam_ptr->ExposureTime.GetValue());
				// convert to nanoseconds
				exp_time *= 1000.0;
				// add half exposure time to the trigger time
				image_stamp += ros::Duration(0, exp_time/2.0);
			}
			ROS_INFO("Image stamp for %s: %d.%d", m_cam_name.c_str(), image_stamp.sec, image_stamp.nsec);
			
			if (image->IsIncomplete())
			{
				ROS_ERROR("Blackfly nodelet : Image retrieval failed : image incomplete");
				return;
			}

			if(m_cam_pub_ptr->getNumSubscribers() > 0)
			{
				int height = image->GetHeight();
				int width = image->GetWidth();
				int stride = image->GetStride();
				int bits_per_px = image->GetBitsPerPixel();
				PixelFormatEnums pix_format = image->GetPixelFormat();
				if(pix_format == PixelFormat_BGR8)
				{
					sensor_msgs::fillImage(*image_msg, sensor_msgs::image_encodings::BGR8, 
											height, width, stride,
											image->GetData());
				}
				else if(pix_format == PixelFormat_Mono8)
				{
					sensor_msgs::fillImage(*image_msg, sensor_msgs::image_encodings::MONO8, 
											height, width, stride,
											image->GetData());
				}
				else
				{
					ROS_ERROR("Unknown pixel format");
					return;
				}
				image_msg->header.frame_id = m_cam_name;
				image_msg->header.stamp = image_stamp;
				
				// setup the camera info object
				sensor_msgs::CameraInfo::Ptr cam_info_msg = boost::make_shared<sensor_msgs::CameraInfo>(sensor_msgs::CameraInfo(m_c_info_mgr_ptr->getCameraInfo()));
				cam_info_msg->header.frame_id = m_cam_name;
				cam_info_msg->header.stamp = image_msg->header.stamp;

				// publish the image
				m_cam_pub_ptr->publish(*image_msg, *cam_info_msg, image_msg->header.stamp);
			}
			image->Release();
		}
		void config_all_chunk_data()
		{
			INodeMap &node_map = m_cam_ptr->GetNodeMap();
			CBooleanPtr ptrChunkModeActive = node_map.GetNode("ChunkModeActive");
			if (!IsAvailable(ptrChunkModeActive) || !IsWritable(ptrChunkModeActive))
			{
				ROS_ERROR("Blackfly Nodelet: Unable to activate chunk mode. Timestamps not compensating for exp time");
				return;
			}
			ptrChunkModeActive->SetValue(true);
			// Retrieve the selector node
			CEnumerationPtr ptrChunkSelector = node_map.GetNode("ChunkSelector");

			if (!IsAvailable(ptrChunkSelector) || !IsReadable(ptrChunkSelector))
			{
				ROS_ERROR("Blackfly Nodelet: Unable to activate chunk mode. Timestamps not compensating for exp time");
				return;
			}
			// Retrieve entries
        	NodeList_t entries;
			ptrChunkSelector->GetEntries(entries);
			for (size_t i = 0; i < entries.size(); i++)
			{
				// Select entry to be enabled
				CEnumEntryPtr ptrChunkSelectorEntry = entries.at(i);

				// Go to next node if problem occurs
				if (!IsAvailable(ptrChunkSelectorEntry) || !IsReadable(ptrChunkSelectorEntry))
				{
					continue;
				}

				ptrChunkSelector->SetIntValue(ptrChunkSelectorEntry->GetValue());
				std::string chunk_entry_name = ptrChunkSelectorEntry->GetSymbolic().c_str();
				//ROS_INFO("Blackfly Nodelet: Enabling %s", chunk_entry_name.c_str());

				// Retrieve corresponding boolean
				CBooleanPtr ptrChunkEnable = node_map.GetNode("ChunkEnable");
				// Enable the boolean, thus enabling the corresponding chunk data
				if (!IsAvailable(ptrChunkEnable))
				{
					ROS_WARN("Blackfly Nodelet: Chunk Data: %s not available", chunk_entry_name.c_str());
				}
				else if (ptrChunkEnable->GetValue())
				{
					//ROS_INFO("Blackfly Nodelet: Chunk Data: %s enabled", chunk_entry_name.c_str());
				}
				else if (IsWritable(ptrChunkEnable))
				{
					ptrChunkEnable->SetValue(true);
					//ROS_INFO("Blackfly Nodelet: Chunk Data: %s enabled", chunk_entry_name.c_str());
				}
				else
				{
					ROS_WARN("Blackfly Nodelet: Chunk Data: %s not writable", chunk_entry_name.c_str());
				}
			}
			ROS_INFO("Blackfly Nodelet: Successfully Configured Chunk Data");
		}
		CameraPtr m_cam_ptr;
	private:
		sensor_msgs::ImagePtr image_msg;
		DeviceEventHandler* m_device_event_handler_ptr;
		boost::shared_ptr<camera_info_manager::CameraInfoManager> m_c_info_mgr_ptr;
		image_transport::CameraPublisher *m_cam_pub_ptr;
		std::string m_cam_name;
		bool m_exp_time_comp_flag = false;
		int img_count;
};
#endif //IMG_EVENT_HANDLER_