#include <vector>
#include <string>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "Spinnaker.h"
#include "image_event_handler.h"
#include "device_event_handler.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <synchronizer/ReadySignal.h>

using namespace Spinnaker;

struct camera_settings
{
	camera_settings()
	{
		cam_name = "None";
		cam_info_path = "None";
		mono = false;
		is_triggered = false;
		fps = 20.0;
		is_auto_exp = true;
		max_auto_exp_time = 30000.0;
		min_auto_exp_time = 50.0;
		fixed_exp_time = 5000.0;
		auto_gain = true;
		gain = 1.0;
		enable_gamma = true;
		gamma = 1.0;
		exp_comp_flag = false;
	}
	camera_settings(std::string cam_name_p, std::string cam_info_path_p, bool mono_p, bool is_triggered_p, float fps_p,
					bool is_auto_exp_p, float max_exp_p, float min_exp_p, float fixed_exp_p,
					bool auto_gain_p, float gain_p, float max_gain_p, float min_gain_p, bool enable_gamma_p,
					float gamma_p, int binning_p, int binning_mode_p, int lighting_mode_p, int auto_exposure_priority_p, bool exp_comp_flag_p)
	{
		cam_name = cam_name_p;
		cam_info_path = cam_info_path_p;
		mono = mono_p;
		is_triggered = is_triggered_p;
		fps = fps_p;
		is_auto_exp = is_auto_exp_p;
		max_auto_exp_time = max_exp_p;
		min_auto_exp_time = min_exp_p;
		fixed_exp_time = fixed_exp_p;
		auto_gain = auto_gain_p;
		gain = gain_p;
		max_gain = max_gain_p;
		min_gain = min_gain_p;
		enable_gamma = enable_gamma_p;
		gamma = gamma_p;
		binning = binning_p;
		binning_mode = binning_mode_p;
		lighting_mode = lighting_mode_p;
		auto_exposure_priority = auto_exposure_priority_p;
		exp_comp_flag = exp_comp_flag_p;
	}
	std::string cam_name;
	std::string cam_info_path;
	bool mono;
	bool is_triggered;
	float fps;
	bool is_auto_exp;
	float max_auto_exp_time;
	float min_auto_exp_time;
	float fixed_exp_time;
	bool auto_gain;
	float gain;
	float min_gain;
	float max_gain;
	bool enable_gamma;
	float gamma;
	int binning;
	int binning_mode;
	int lighting_mode;
	int auto_exposure_priority;
	bool exp_comp_flag;
};

class blackfly_camera
{
public:
	blackfly_camera(camera_settings settings, CameraPtr cam_ptr, uint8_t id)
	{
		// save the camera pointer and the settings object
		m_cam_ptr = cam_ptr;
		m_cam_settings = settings;

		// create a new node handle
		ros::NodeHandle nh(m_cam_settings.cam_name);

		// setup ros image transport
		m_image_transport_ptr = new image_transport::ImageTransport(nh);
		m_cam_pub = m_image_transport_ptr->advertiseCamera(m_cam_settings.cam_name, 10);
		m_cam_info_mgr_ptr = boost::make_shared<camera_info_manager::CameraInfoManager>(nh, m_cam_settings.cam_name, m_cam_settings.cam_info_path);
		m_cam_info_mgr_ptr->loadCameraInfo(m_cam_settings.cam_info_path);

		// setup the camera
		setup_camera();

		// create event handlers
		m_device_event_handler_ptr = new DeviceEventHandler(m_cam_ptr);
		m_image_event_handler_ptr = new ImageEventHandler(m_cam_settings.cam_name, m_cam_ptr, &m_cam_pub, m_cam_info_mgr_ptr, m_device_event_handler_ptr, m_cam_settings.exp_comp_flag, id);

		// register event handlers
		m_cam_ptr->RegisterEvent(*m_device_event_handler_ptr);
		m_cam_ptr->RegisterEvent(*m_image_event_handler_ptr);

		m_cam_ptr->BeginAcquisition();
		
		// Tell server that camera is ready!
		ros::NodeHandle n;
		ros::ServiceClient service = n.serviceClient<synchronizer::ReadySignal>("ready_signal", false);
		synchronizer::ReadySignal srv;
		srv.request.id = id;
		while (!service.call(srv)){
			ROS_WARN_ONCE("Service not ready? Waiting for response on camera ready signal");
		};
	}
	~blackfly_camera()
	{
		if (m_cam_ptr->IsValid())
		{
			m_cam_ptr->EndAcquisition();
			m_cam_ptr->UnregisterEvent(*m_image_event_handler_ptr);
			m_cam_ptr->UnregisterEvent(*m_device_event_handler_ptr);
			delete m_image_event_handler_ptr;
			delete m_device_event_handler_ptr;
			m_cam_ptr->DeInit();
			std::free(user_buffer);
		}
	}
	// This function sets the internal buffersize of spinnaker -> By default it allocates memory based on the frame rate of the camera.
	// This may require very large memory when using multiple cameras, and may cause additional issues.
	void set_buffer_size(unsigned int buff_size)
	{
		// Retrieve Stream Parameters device nodemap
		Spinnaker::GenApi::INodeMap &sNodeMap = m_cam_ptr->GetTLStreamNodeMap();
		// Retrieve Buffer Handling Mode Information
		CEnumerationPtr ptrHandlingMode = sNodeMap.GetNode("StreamBufferHandlingMode");
		if (!IsAvailable(ptrHandlingMode) || !IsWritable(ptrHandlingMode))
		{
			ROS_ERROR("Unable to set Buffer Handling mode (node retrieval). Aborting...");
		}
		CEnumEntryPtr ptrHandlingModeEntry = ptrHandlingMode->GetCurrentEntry();
		if (!IsAvailable(ptrHandlingModeEntry) || !IsReadable(ptrHandlingModeEntry))
		{
			ROS_ERROR("Unable to set Buffer Handling mode (Entry retrieval). Aborting...");
		}
		// Set stream buffer Count Mode to manual
		CEnumerationPtr ptrStreamBufferCountMode = sNodeMap.GetNode("StreamBufferCountMode");
		if (!IsAvailable(ptrStreamBufferCountMode) || !IsWritable(ptrStreamBufferCountMode))
		{
			ROS_ERROR("Unable to set Buffer Count Mode (node retrieval). Aborting...");
		}
		CEnumEntryPtr ptrStreamBufferCountModeManual = ptrStreamBufferCountMode->GetEntryByName("Manual");
		if (!IsAvailable(ptrStreamBufferCountModeManual) || !IsReadable(ptrStreamBufferCountModeManual))
		{
			ROS_ERROR("Unable to set Buffer Count Mode entry (Entry retrieval). Aborting...");
		}
		ptrStreamBufferCountMode->SetIntValue(ptrStreamBufferCountModeManual->GetValue());
		// Retrieve and modify Stream Buffer Count
		CIntegerPtr ptrBufferCount = sNodeMap.GetNode("StreamBufferCountManual");
		if (!IsAvailable(ptrBufferCount) || !IsWritable(ptrBufferCount))
		{
			ROS_ERROR("Unable to set Buffer Count (Integer node retrieval). Aborting...");
		}
		// Display Buffer Info
		ptrBufferCount->SetValue(buff_size);
	}
	void setup_camera()
	{
		try
		{
			m_cam_ptr->DeviceReset;
			// initialize the camera
			m_cam_ptr->Init();
			
			m_cam_ptr->AcquisitionStop();

			// Set up pixel format
			if (m_cam_settings.mono)
			{
				m_cam_ptr->PixelFormat = PixelFormat_Mono8;
			}
			else
			{
				m_cam_ptr->PixelFormat = PixelFormat_BGR8;
			}
			m_cam_ptr->BinningVertical = m_cam_settings.binning;
			m_cam_ptr->BinningHorizontal = m_cam_settings.binning;

			// Set vertical flip because camera is upside down!
			m_cam_ptr->ReverseY = true;
			m_cam_ptr->ReverseX = true;

			// set binning type 0=Average, 1=Sum
			if (m_cam_settings.binning_mode == 0)
			{
				m_cam_ptr->BinningHorizontalMode.SetValue(BinningHorizontalModeEnums::BinningHorizontalMode_Average);
				m_cam_ptr->BinningVerticalMode.SetValue(BinningVerticalModeEnums::BinningVerticalMode_Average);
			}
			else if (m_cam_settings.binning_mode == 1)
			{
				m_cam_ptr->BinningHorizontalMode.SetValue(BinningHorizontalModeEnums::BinningHorizontalMode_Sum);
				m_cam_ptr->BinningVerticalMode.SetValue(BinningVerticalModeEnums::BinningVerticalMode_Sum);
			}

			// set lighting type 0=Normal, 1=Backlight, 2=Frontlight
			if (m_cam_settings.lighting_mode == 1)
			{
				m_cam_ptr->AutoExposureLightingMode.SetValue(AutoExposureLightingModeEnums::AutoExposureLightingMode_Backlight);
			}
			else if (m_cam_settings.lighting_mode == 2)
			{
				m_cam_ptr->AutoExposureLightingMode.SetValue(AutoExposureLightingModeEnums::AutoExposureLightingMode_Frontlight);
			}

			// set acquisition mode, Continuous instead of single frame or burst modes
			m_cam_ptr->AcquisitionMode = AcquisitionMode_Continuous;

			// setup exposure
			if (m_cam_settings.is_auto_exp)
			{
				m_cam_ptr->ExposureAuto = ExposureAuto_Continuous;
				m_cam_ptr->AutoExposureExposureTimeUpperLimit = m_cam_settings.max_auto_exp_time;
				m_cam_ptr->AutoExposureExposureTimeLowerLimit = m_cam_settings.min_auto_exp_time;
			}
			else
			{
				m_cam_ptr->ExposureAuto = ExposureAuto_Off;
				m_cam_ptr->ExposureTime = m_cam_settings.fixed_exp_time;
			}
			// setup gain
			m_cam_ptr->GainAuto = GainAuto_Off;
			if (m_cam_settings.auto_gain)
			{
				m_cam_ptr->GainAuto = GainAuto_Continuous;
				m_cam_ptr->AutoExposureGainUpperLimit = m_cam_settings.max_gain;
				m_cam_ptr->AutoExposureGainLowerLimit = m_cam_settings.min_gain;
			}
			else
			{
				m_cam_ptr->Gain.SetValue(m_cam_settings.gain);
			}
			// setup gamma
			if (m_cam_settings.enable_gamma)
			{
				m_cam_ptr->GammaEnable = true;
				m_cam_ptr->Gamma.SetValue(m_cam_settings.gamma);
			}
			else
			{
				m_cam_ptr->GammaEnable = false;
			}
			// setup trigger parameters
			if (m_cam_settings.is_triggered)
			{
				m_cam_ptr->TriggerMode = TriggerMode_Off;
				m_cam_ptr->TriggerSource = TriggerSource_Line0;
				// m_cam_ptr->TriggerSource = TriggerSource_Line1;
				// m_cam_ptr->TriggerSource = TriggerSource_Line2;
				// m_cam_ptr->TriggerSource = TriggerSource_Line2;
				// m_cam_ptr->TriggerSource = TriggerSource_Counter0End;
				m_cam_ptr->TriggerActivation = TriggerActivation_RisingEdge;
				m_cam_ptr->TriggerMode = TriggerMode_On;
			}
			else
			{
				m_cam_ptr->TriggerMode = TriggerMode_Off;
				m_cam_ptr->AcquisitionFrameRateEnable = true;
				m_cam_ptr->AcquisitionFrameRate = m_cam_settings.fps;
			}
			m_cam_ptr->ExposureMode = ExposureMode_Timed;
			set_buffer_size(5);
		}
		catch (Spinnaker::Exception &ex)
		{
			ROS_ERROR("ERROR SETTING CAMERA SETTINGS!!!");
			std::cout << "Error: " << ex.what() << std::endl;
			std::cout << "Error code " << ex.GetError() << " raised in function " << ex.GetFunctionName() << " at line " << ex.GetLineNumber() << "." << std::endl;
		}
	}

private:
	size_t total_size = sizeof(int8_t) * 1024;
	void *user_buffer = malloc(total_size);
	// void* buffer = nullptr;
	CameraPtr m_cam_ptr;
	camera_settings m_cam_settings;
	ImageEventHandler *m_image_event_handler_ptr;
	DeviceEventHandler *m_device_event_handler_ptr;
	image_transport::ImageTransport *m_image_transport_ptr;
	image_transport::CameraPublisher m_cam_pub;
	boost::shared_ptr<camera_info_manager::CameraInfoManager> m_cam_info_mgr_ptr;
};