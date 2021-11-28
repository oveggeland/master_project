#ifndef DEV_EVENT_HANDLER_
#define DEV_EVENT_HANDLER_
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream> 
#include <ros/ros.h>
#include <deque>
#include <mutex>

class TimeStampSynchronizer 
{
	public:
		// Initilization
		TimeStampSynchronizer(int p_n_cams){
			n_cams = p_n_cams;
		}

		// Set new time stamp TODO: Only allow one camera to do this
		void set_time_stamp(ros::Time new_stamp){
			synch_mutex.lock();

			if (stamps_retrieved >= n_cams){
				ROS_DEBUG("Setting new time stamp %d", new_stamp.nsec);
				last_time_stamp = new_stamp;
				stamps_retrieved = 0;
			}
			
			synch_mutex.unlock();
		}

		// Retrieve current time stamp
		ros::Time get_time_stamp(){
			synch_mutex.lock();
			
			if (stamps_retrieved >= n_cams){
				last_time_stamp = ros::Time(0, 0);
				stamps_retrieved = 0;
			}
			else{
				stamps_retrieved ++;
			}

			synch_mutex.unlock();
			return last_time_stamp;
		}

	private:
		std::mutex synch_mutex; 
		int n_cams;
		ros::Time last_time_stamp = ros::Time(0,0);
		int stamps_retrieved = 0;
};


using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

class DeviceEventHandler : public DeviceEvent
{
	public:
		DeviceEventHandler(CameraPtr cam_ptr, TimeStampSynchronizer* synch_ptr)
		{
			// save the camera pointer
			m_cam_ptr = cam_ptr;
			// Save synch pointer
			m_synch_ptr = synch_ptr;
			// get the GENAPI node map
			INodeMap &node_map = m_cam_ptr->GetNodeMap();
			try
			{
				CEnumerationPtr ptrEventSelector = node_map.GetNode("EventSelector");
				if (!IsAvailable(ptrEventSelector) || !IsReadable(ptrEventSelector))
				{
					ROS_ERROR("Blackfly Nodelet: Unable to retrieve event selector entries. Aborting");
					return;
				}
				NodeList_t entries;
				ptrEventSelector->GetEntries(entries);
				for (unsigned int i = 0; i < entries.size(); i++)
				{
					// Select entry on selector node
					CEnumEntryPtr ptrEnumEntry = entries.at(i);
					if(ptrEnumEntry->GetDisplayName() == "Exposure End")
					{
						if (!IsAvailable(ptrEnumEntry) || !IsReadable(ptrEnumEntry))
						{
							// Skip if node fails
							continue;
						}
						ptrEventSelector->SetIntValue(ptrEnumEntry->GetValue());
						// Retrieve event notification node (an enumeration node)
						CEnumerationPtr ptrEventNotification = node_map.GetNode("EventNotification");
						if (!IsAvailable(ptrEventNotification) || !IsWritable(ptrEventNotification))
						{
							continue;
						}
						// Retrieve entry node to enable device event
						CEnumEntryPtr ptrEventNotificationOn = ptrEventNotification->GetEntryByName("On");
						if (!IsAvailable(ptrEventNotification) || !IsReadable(ptrEventNotification))
						{
							continue;
						}
						ptrEventNotification->SetIntValue(ptrEventNotificationOn->GetValue());
					}
				}
			}
			catch (Spinnaker::Exception &e)
			{
				ROS_FATAL("Blackfly Nodelet: Failed to configure device event handler");
			}
		}
		~DeviceEventHandler()
		{
			// this is how the spinnaker examples release the camera pointer and allow the nodelet to exit cleanly
			m_cam_ptr = nullptr;
		}
		void OnDeviceEvent(gcstring eventName)
		{
			if(eventName == "EventExposureEnd")
			{
				// Feed frame time to synchronizer
				m_synch_ptr->set_time_stamp(ros::Time::now());
			}
		}
		ros::Time get_last_exposure_end()
		{
			// retrieve time stamp from synchronizer
			return m_synch_ptr->get_time_stamp();
		}
	private:
		// initialize the timestamp member to 0.0 to indicate it has not been set
		ros::Time stamp;
		// Camera pointer to spinnaker camera object (used to get the current exposure time)
		CameraPtr m_cam_ptr;
		// For synchronizing timestamps between cameras
		TimeStampSynchronizer* m_synch_ptr;
};
#endif // DEV_EVENT_HANDLER