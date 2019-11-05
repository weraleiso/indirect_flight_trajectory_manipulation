// clang-format off
// source: https://github.com/AndreGilerson/rviz_vive_plugin
// source: https://blog.ybalrid.info/2016/09/12/using-ogres-opengl-renderer-with-the-openvr-api-steamvr-htc-vive-sdk/
// clang-format on
#include "vive_compositor.h"

namespace rviz_vive_compositor_plugin {

inline Ogre::Matrix4 ViveCompositor::getMatrix4FromSteamVRMatrix34(
    const vr::HmdMatrix34_t &mat) {
  return Ogre::Matrix4{mat.m[0][0], mat.m[0][1], mat.m[0][2], mat.m[0][3],
                       mat.m[1][0], mat.m[1][1], mat.m[1][2], mat.m[1][3],
                       mat.m[2][0], mat.m[2][1], mat.m[2][2], mat.m[2][3],
                       0.0f,        0.0f,        0.0f,        1.0f};
}

inline Ogre::Vector3 ViveCompositor::getTrackedHMDTranslation() {
  // Extract translation vector from the matrix
  return hmdAbsoluteTransform.getTrans();
}

inline Ogre::Quaternion ViveCompositor::getTrackedHMDOrientation() {
  return hmdAbsoluteTransform.extractQuaternion();
}

// This function is from VALVe
std::string ViveCompositor::GetTrackedDeviceString(
    vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice,
    vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError) {
  uint32_t unRequiredBufferLen =
      pHmd->GetStringTrackedDeviceProperty(unDevice, prop, NULL, 0, peError);
  if (unRequiredBufferLen == 0) return "";

  char *pchBuffer = new char[unRequiredBufferLen];
  unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(
      unDevice, prop, pchBuffer, unRequiredBufferLen, peError);
  std::string sResult = pchBuffer;
  delete[] pchBuffer;

  return sResult;
}

inline vr::EVREye ViveCompositor::getEye(oovrEyeType eye) {
  return eye == left ? vr::Eye_Left : vr::Eye_Right;
}

void ViveCompositor::processCompressedImage(
    const sensor_msgs::ImageConstPtr &msg, oovrEyeType eye) {
  if (!enableStereoCameraView->getBool()) {
    return;
  }

  // Identify the texture format
  Ogre::PixelFormat pixelFormat = Ogre::PF_R8G8B8;

  // This completely fails, probably because of using compressed image
  /*std::string msgEncoding(msg->encoding);
  if (msgEncoding.compare(sensor_msgs::image_encodings::RGB8)) {
    ROS_WARN("Pixel format RGB8");
    pixelFormat = Ogre::PF_R8G8B8;
  } else if (msgEncoding.compare(sensor_msgs::image_encodings::RGBA8)) {
    ROS_WARN("Pixel format RGBA8");
    pixelFormat = Ogre::PF_R8G8B8A8;
  }*/

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Size size = cv_ptr->image.size();

  // Initialize the Ogre texture, if it is the first time
  if (stereoCameraTextures[eye].isNull()) {
    std::string textureName("stereo_camera_" + std::to_string(eye));
    stereoCameraTextures[eye] = textureManager->createManual(
        textureName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D, Ogre::uint(size.width), Ogre::uint(size.height), 0,
        pixelFormat, Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE, nullptr, gamma);

    stereoCameraTexturesGLID[eye] =
        static_cast<Ogre::GLTexture *>(
            textureManager->getByName(textureName).getPointer())
            ->getGLID();

    vrTextures[eye] = {(void *)(long)stereoCameraTexturesGLID[eye], API,
                       vr::ColorSpace_Gamma};

    ROS_WARN("Stereo camera texture for %d. eye created.", eye);
  }

  // Perhaps needed
  // stereoCameraTextures[eye]->unload();
  // stereoCameraTextures[eye]->loadImage(image);

  Ogre::PixelBox pb(size.width, size.height, 1, Ogre::PF_R8G8B8,
                    cv_ptr->image.data);
  Ogre::HardwarePixelBufferSharedPtr buffer =
      stereoCameraTextures[eye]->getBuffer();
  buffer->blitFromMemory(pb);
}

void ViveCompositor::sub_left_stereo_camera_callback(
    const sensor_msgs::ImageConstPtr &msg) {
  processCompressedImage(msg, left);
}

void ViveCompositor::sub_right_stereo_camera_callback(
    const sensor_msgs::ImageConstPtr &msg) {
  processCompressedImage(msg, right);
}

void ViveCompositor::setupStereoCamera() {
  image_transport::TransportHints hints(
      "compressed", ros::TransportHints(),
      update_nh_);  // I guess I shouldn't use the root nodeHandle?

  sub_left_stereo_camera = imageTransport.subscribe(
      "/reshaping/bebop2camera/camera/left/image_raw", 1,
      &ViveCompositor::sub_left_stereo_camera_callback, this, hints);
  sub_right_stereo_camera = imageTransport.subscribe(
      "/reshaping/bebop2camera/camera/right/image_raw", 1,
      &ViveCompositor::sub_right_stereo_camera_callback, this, hints);

  // Remove rendering of the virtual scene for Vive
  rttTexture[left]->getBuffer()->getRenderTarget()->removeAllViewports();
  rttTexture[right]->getBuffer()->getRenderTarget()->removeAllViewports();
  // Reset the handler for SteamVR textures
  vrTextures[left].handle = (void *)(long)stereoCameraTexturesGLID[left];
  vrTextures[right].handle = (void *)(long)stereoCameraTexturesGLID[right];

  GLBounds.uMin = -0.2f;
  GLBounds.uMax = 1.2f;
  GLBounds.vMin = 1.2f;
  GLBounds.vMax = -0.2f;

  ROS_WARN("Stereo camera subscription created.");
}

void ViveCompositor::setupOgre() {
  // VR Eye cameras
  rvizSceneNode = sceneManager->getRootSceneNode()->createChildSceneNode();

  // Camera for  each eye
  eyeCameras[left] = sceneManager->createCamera("lcam");
  eyeCameras[left]->setAutoAspectRatio(true);
  rvizSceneNode->attachObject(eyeCameras[left]);

  eyeCameras[right] = sceneManager->createCamera("rcam");
  eyeCameras[right]->setAutoAspectRatio(true);
  rvizSceneNode->attachObject(eyeCameras[right]);

  // Get the eyeToHeadTransform (they contain the IPD translation)
  for (uint8_t i = 0; i < 2; i++)
    eyeCameras[i]->setPosition(
        getMatrix4FromSteamVRMatrix34(
            vrSystem->GetEyeToHeadTransform(getEye(oovrEyeType(i))))
            .getTrans());

  // Get the render texture size recomended by the OpenVR API for the current
  // Driver/Display
  unsigned int w, h;
  vrSystem->GetRecommendedRenderTargetSize(&w, &h);

  // Left eye texture
  rttTexture[left] = textureManager->createManual(
      "RTT_TEX_L", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D, w, h, 0, Ogre::PF_R8G8B8, Ogre::TU_RENDERTARGET,
      nullptr, gamma);
  rttTextureGLID[left] =
      static_cast<Ogre::GLTexture *>(
          textureManager->getByName("RTT_TEX_L").getPointer())
          ->getGLID();

  // Right eye texture
  rttTexture[right] = textureManager->createManual(
      "RTT_TEX_R", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D, w, h, 0, Ogre::PF_R8G8B8, Ogre::TU_RENDERTARGET,
      nullptr, gamma);
  rttTextureGLID[right] =
      static_cast<Ogre::GLTexture *>(
          textureManager->getByName("RTT_TEX_R").getPointer())
          ->getGLID();

  // Create viewport for each cameras in each render texture
  rttViewports[left] =
      rttTexture[left]->getBuffer()->getRenderTarget()->addViewport(
          eyeCameras[left]);
  rttViewports[right] =
      rttTexture[right]->getBuffer()->getRenderTarget()->addViewport(
          eyeCameras[right]);

  float nearClippingDistance = 0.1f;
  float farClippingDistance = 1000.0f;

  // Get the couple of matrices
  vr::HmdMatrix44_t prj[2] = {
      vrSystem->GetProjectionMatrix(getEye(left), nearClippingDistance,
                                    farClippingDistance),
      vrSystem->GetProjectionMatrix(getEye(right), nearClippingDistance,
                                    farClippingDistance)};

  // Apply them to the camera
  for (uint8_t eye = 0; eye < 2; eye++) {
    // Need to convert them to Ogre's object
    Ogre::Matrix4 m;
    for (uint8_t i = 0; i < 4; i++) {
      for (uint8_t j = 0; j < 4; j++) {
        m[i][j] = prj[eye].m[i][j];
      }
    }

    // Apply projection matrix
    eyeCameras[eye]->setCustomProjectionMatrix(true, m);
  }

  // This could possibly help with FPS issues
  // vr::VRCompositor()->ForceInterleavedReprojectionOn(true);

  // Declare the textures for SteamVR
  vrTextures[left] = {(void *)(long)rttTextureGLID[left], API,
                      vr::ColorSpace_Gamma};
  vrTextures[right] = {(void *)(long)rttTextureGLID[right], API,
                       vr::ColorSpace_Gamma};

  ROS_WARN("Texture setup completed.");
}

void ViveCompositor::onInitialize() {
  ROS_WARN("Vive compositor initializing");

  sceneManager = scene_manager_;
  displayContext = context_;

  // Create theses textures in OpenGL and get their OpenGL ID
  textureManager = static_cast<Ogre::GLTextureManager *>(
      Ogre::TextureManager::getSingletonPtr());

  // Setup virtual scene
  setupOgre();

  // Set the OpenGL texture geometry
  GLBounds = {};
  GLBounds.uMin = 0;
  GLBounds.uMax = 1;
  GLBounds.vMin = 1;
  GLBounds.vMax = 0;

  ROS_WARN("Created the ViveCompositor object");
}

void ViveCompositor::update(float wall_dt, float ros_dt) {
  tf::StampedTransform tf_world_to_camera_drone;
  tf::Vector3 pos_vive;
  tf::Quaternion quat_vive;

  try {
    tf_listener_world_to_camera_drone.lookupTransform(
        "/world", "reshaping/bebop2camera/htc_vive_hmd_base", ros::Time(0),
        tf_world_to_camera_drone);
    pos_vive = tf_world_to_camera_drone.getOrigin();
    quat_vive = tf_world_to_camera_drone.getRotation();

    cameraPosition.x = pos_vive.getX();
    cameraPosition.y = pos_vive.getY();
    cameraPosition.z = pos_vive.getZ();

    cameraRotation.x = -quat_vive.getX();
    cameraRotation.y = -quat_vive.getY();
    cameraRotation.z = -quat_vive.getZ();
    cameraRotation.w = quat_vive.getW();
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  vr::VRCompositor()->WaitGetPoses(trackedPoses, vr::k_unMaxTrackedDeviceCount,
                                   nullptr, 0);

  vr::TrackedDevicePose_t hmdPose;
  if ((hmdPose = trackedPoses[vr::k_unTrackedDeviceIndex_Hmd]).bPoseIsValid)
    hmdAbsoluteTransform =
        getMatrix4FromSteamVRMatrix34(hmdPose.mDeviceToAbsoluteTracking);

  // Update the eye rig tracking to make the eyes match yours
  rvizSceneNode->setPosition(cameraPosition);
  // Use this, if you want to take the HMD's location into account
  // rvizSceneNode->setPosition(cameraPosition +
  //                           eyeRotation * getTrackedHMDTranslation());

  static Ogre::Radian halfPi(M_PI / 2.0);
  static Ogre::Quaternion rotationAroundX(halfPi, Ogre::Vector3(1, 0, 0));
  static Ogre::Quaternion RotationAroundZ(halfPi, Ogre::Vector3(0, 1, 0));
  static Ogre::Quaternion eyeRotationCorrection(rotationAroundX * RotationAroundZ);

  Ogre::Quaternion hdmRotation = getTrackedHMDOrientation();

  // Multiply the received camera rotation, the eye rotation correction from
  // difference in coordinate systems and hmd rotation
  if (compensateDroneRotationProperty->getBool())
  {
    // Compensate for camera drone movement
    Ogre::Quaternion inverseYaw(cameraRotation.w, cameraRotation.x,cameraRotation.y, -cameraRotation.z);
    Ogre::Quaternion hmdPitchRoll(hdmRotation.w, hdmRotation.x, 0,hdmRotation.z);
    rvizSceneNode->setOrientation(inverseYaw * eyeRotationCorrection * hmdPitchRoll);
  }
  else
  {
    // Do normal rotation in scene
    // rvizSceneNode->setOrientation(cameraRotation * eyeRotationCorrection * hdmRotation);

    // Consider YAW roation of HMD based on rotation of UAV in world coords
    Ogre::Quaternion quat_uav_camera(quat_vive.getW(),quat_vive.getX(),quat_vive.getY(),quat_vive.getZ());

    double view_angle_saturation = 1.75;
    tf::Matrix3x3 mat_uav_camera(tf::Quaternion(hdmRotation.x,hdmRotation.y,hdmRotation.z,hdmRotation.w));
    double head_yaw, head_pitch, head_roll;
    mat_uav_camera.getRPY(head_roll, head_pitch, head_yaw);
    /*if (head_pitch < -1.0) head_pitch = -1.0;
    if (head_pitch > 1.0) head_pitch = 1.0;
    if (head_roll > -2.0) head_roll = -2.0;
    if (head_roll < 2.0) head_roll = 2.0;*/
    //ROS_INFO("SAT RPY: %3.3f %3.3f %3.3f",head_roll,head_pitch,head_yaw);
    tf::Quaternion q_rot;
    q_rot.setRPY(head_roll,head_pitch,head_yaw); // keep yaw zero
    Ogre::Quaternion quat_with_sat;
    quat_with_sat.w=q_rot.getW();
    quat_with_sat.x=q_rot.getX();
    quat_with_sat.y=q_rot.getY();
    quat_with_sat.z=q_rot.getZ();

    //Ogre::Quaternion quat_without_sat=quat_uav_camera * eyeRotationCorrection * quat_with_sat;
    Ogre::Quaternion quat_without_sat=quat_uav_camera * eyeRotationCorrection * hdmRotation;

    rvizSceneNode->setOrientation(quat_without_sat);
  }

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0, 0, 0));

  // Take into account the difference in coordinate systems
  transform.setRotation(tf::Quaternion(-hdmRotation.z, -hdmRotation.x,
                                       hdmRotation.y, hdmRotation.w));

  tfBroadcaster.sendTransform(tf::StampedTransform(
      transform, ros::Time::now(), "reshaping/bebop2camera/htc_vive_hmd_base",
      "reshaping/bebop2camera/htc_vive_hmd"));

  // Check if vrTextures set. When using the stereo cameras they will probably
  // not be in the first iteration
  if (vrTextures[left].handle && vrTextures[right].handle) {
    // Submit the textures to the SteamVR compositor
    vr::VRCompositor()->Submit(getEye(left), &vrTextures[left], &GLBounds);
    vr::VRCompositor()->Submit(getEye(right), &vrTextures[right], &GLBounds);
  }
  // vr::VRCompositor()->PostPresentHandoff();
}

void ViveCompositor::reset() {}

void ViveCompositor::enableStereoCameraViewChanged() {
  ROS_WARN("Stereo camera view changed to %s",
           enableStereoCameraView->getBool() ? "true" : "false");
  if (enableStereoCameraView->getBool()) {
    setupStereoCamera();
  } else {
    sub_left_stereo_camera.shutdown();
    sub_right_stereo_camera.shutdown();
    // Create viewport for each cameras in each render texture
    rttViewports[left] =
        rttTexture[left]->getBuffer()->getRenderTarget()->addViewport(
            eyeCameras[left]);
    rttViewports[right] =
        rttTexture[right]->getBuffer()->getRenderTarget()->addViewport(
            eyeCameras[right]);

    vrTextures[left].handle = (void *)(long)rttTextureGLID[left];
    vrTextures[right].handle = (void *)(long)rttTextureGLID[right];

    GLBounds.uMin = 0;
    GLBounds.uMax = 1;
    GLBounds.vMin = 1;
    GLBounds.vMax = 0;
  }
}

ViveCompositor::ViveCompositor() : imageTransport(update_nh_) {
  ROS_WARN("Vive compositor spawned");

  vrSystem = nullptr;
  hmdError = vr::EVRInitError::VRInitError_None;
  gamma = false;
  API = vr::TextureType_OpenGL;
  hmdAbsoluteTransform = Ogre::Matrix4();
  rvizSceneNode = nullptr;

  rttTexture[left].setNull();
  rttTexture[right].setNull();

  rttTextureGLID[left] = 0;
  rttTextureGLID[right] = 0;
  stereoCameraTexturesGLID[left] = 0;
  stereoCameraTexturesGLID[right] = 0;

  rttViewports[left] = nullptr;
  rttViewports[right] = nullptr;

  vrTextures[left] = {};
  vrTextures[right] = {};
  GLBounds = {};

  compensateDroneRotationProperty = new rviz::BoolProperty(
      "Compensate drone rotation", true,
      "If checked will compensate for the camera drone movement.", this);

  enableStereoCameraView =
      new rviz::BoolProperty("Enable stereo camera view", false,
                             "If checked will render the views from two stereo "
                             "cameras instead of the virtual scene.",
                             this, SLOT(enableStereoCameraViewChanged()));

  // Initialize OpenVR
  vrSystem =
      vr::VR_Init(&hmdError, vr::EVRApplicationType::VRApplication_Scene);
  if (hmdError != vr::VRInitError_None)  // Check for errors
    switch (hmdError) {
      default:
        ROS_WARN(
            "Error: failed OpenVR VR_Init. Undescribed error when initalizing "
            "the OpenVR Render object: %d",
            hmdError);
        exit(-1);

      case vr::VRInitError_Init_HmdNotFound:
      case vr::VRInitError_Init_HmdNotFoundPresenceFailed:
        ROS_WARN(
            "Error: cannot find HMD. OpenVR cannot find HMD.\nPlease install "
            "SteamVR and launchj it, and verify HMD USB and HDMI connection");
        exit(-2);
    }
  // Check if VRCompositor is present
  if (!vr::VRCompositor()) {
    ROS_WARN(
        "Error: failed to init OpenVR VRCompositor. Failed to initialize the "
        "VR Compositor");
    exit(-3);
  }

  // Get Driver and Display information
  strDriver = GetTrackedDeviceString(vrSystem, vr::k_unTrackedDeviceIndex_Hmd,
                                     vr::Prop_TrackingSystemName_String);
  strDisplay = GetTrackedDeviceString(vrSystem, vr::k_unTrackedDeviceIndex_Hmd,
                                      vr::Prop_SerialNumber_String);

  ROS_WARN("Driver: %s, display: %s", strDriver.c_str(), strDisplay.c_str());
  std::cerr << "Driver : " << strDriver;
  std::cerr << "Display : " << strDisplay;
}

ViveCompositor::~ViveCompositor() {
  // Should disconnect the device and remove the IVRSystem
  vr::VR_Shutdown();

  delete eyeCameras[left];
  delete eyeCameras[right];
  rttTexture[left]->getBuffer()->getRenderTarget()->removeAllViewports();
  rttTexture[right]->getBuffer()->getRenderTarget()->removeAllViewports();
  delete rvizSceneNode;
  delete textureManager;
}

}  // namespace rviz_vive_compositor_plugin
   // clang-format off
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_vive_compositor_plugin::ViveCompositor, rviz::Display)
