
#ifndef VIVE_COMPOSITOR_H
#define VIVE_COMPOSITOR_H

#ifndef Q_MOC_RUN
#include "rviz/display.h"
#endif

#include <QObject>

#include <geometry_msgs/PoseStamped.h>
#include <rviz/display.h>

#include <openvr.h>
#include <openvr_capi.h>

#include <OGRE/RenderSystems/GL/OgreGLRenderSystem.h>
#include <OGRE/RenderSystems/GL/OgreGLTexture.h>
#include <OGRE/RenderSystems/GL/OgreGLTextureManager.h>

#include <OGRE/Ogre.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/render_system.h>
#include <rviz/ogre_helpers/render_widget.h>
#include <rviz/render_panel.h>
#include <rviz/view_manager.h>
#include <rviz/window_manager_interface.h>

#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <rviz/display.h>
#include <sensor_msgs/CompressedImage.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

namespace rviz_vive_compositor_plugin {

class ViveCompositor : public rviz::Display {
  Q_OBJECT

  // Overrides from rviz::Display
  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

 public:
  ViveCompositor();
  virtual ~ViveCompositor();

 private:
  typedef enum { left, right } oovrEyeType;

  Ogre::Camera* eyeCameras[2];

  /// main OpenVR object
  vr::IVRSystem* vrSystem;

  /// Error handeling vaiable
  vr::HmdError hmdError;

  /// EyeCamera render texures
  Ogre::TexturePtr rttTexture[2];

  /// OpenGL "id" of the render textures
  GLuint rttTextureGLID[2];

  /// EyeCameraViewport
  Ogre::Viewport* rttViewports[2];

  /// Use hardware gamma correction
  bool gamma;

  /// API handler, should be initialized to "OpenGL"
  vr::ETextureType API;

  /// OpenVR texture handlers
  vr::Texture_t vrTextures[2];

  /// OpenVR device strings
  std::string strDriver, strDisplay;

  /// Geometry of an OpenGL texture
  vr::VRTextureBounds_t GLBounds;

  /// Array of tracked poses
  vr::TrackedDevicePose_t trackedPoses[vr::k_unMaxTrackedDeviceCount];

  /// Transform that corespond to the HMD tracking
  Ogre::Matrix4 hmdAbsoluteTransform;

  /// Camera Rig that holds the 2 cameras on the same plane
  Ogre::SceneNode* rvizSceneNode;

  /// Camera/Eye position provided by the PoseStamped message
  Ogre::Vector3 cameraPosition;
  Ogre::Quaternion cameraRotation;

  /// The render objects, provided by parent RVIZ window
  rviz::DisplayContext* displayContext;
  Ogre::SceneManager* sceneManager;

  /// OpenGL texture manager
  Ogre::GLTextureManager* textureManager;

  /// Support functions
  Ogre::Vector3 getTrackedHMDTranslation();
  Ogre::Quaternion getTrackedHMDOrientation();
  Ogre::Matrix4 getMatrix4FromSteamVRMatrix34(const vr::HmdMatrix34_t& mat);
  vr::EVREye getEye(oovrEyeType eye);
  std::string GetTrackedDeviceString(
      vr::IVRSystem* pHmd, vr::TrackedDeviceIndex_t unDevice,
      vr::TrackedDeviceProperty prop,
      vr::TrackedPropertyError* peError = __null);

  /// Setup all components
  void setupOgre();
  void setupStereoCamera();

  /// Process incoming messages
  void processMessage(const geometry_msgs::PoseStamped::ConstPtr& msg);

  /// RVIZ translation and orientation checboxes
  /// They are not used for now, but I should add them, if I publish as
  /// open-source
  // rviz::BoolProperty* ignoreViveTranslation;
  // rviz::BoolProperty* ignoreViveOrientation;

  /// TF stuff: we receive the head position via a listener, and broadcast the
  /// rotation
  tf::TransformListener tf_listener_world_to_camera_drone;
  tf::TransformBroadcaster tfBroadcaster;

  /// Enable drone rotation compensation
  rviz::BoolProperty* compensateDroneRotationProperty;

  /// Enable stereo camera view
  rviz::BoolProperty* enableStereoCameraView;

  /// Stereo camera render texures
  Ogre::TexturePtr stereoCameraTextures[2];

  /// OpenGL "id" of the stereo camera render textures
  GLuint stereoCameraTexturesGLID[2];

  /// Stereo camera message subscibers
  image_transport::ImageTransport imageTransport;
  image_transport::Subscriber sub_left_stereo_camera;
  image_transport::Subscriber sub_right_stereo_camera;

  void sub_left_stereo_camera_callback(const sensor_msgs::ImageConstPtr& msg);
  void sub_right_stereo_camera_callback(const sensor_msgs::ImageConstPtr& msg);
  void processCompressedImage(const sensor_msgs::ImageConstPtr& msg,
                              oovrEyeType eye);

 private Q_SLOTS:
  // Callback for change of RVIZ parameter
  void enableStereoCameraViewChanged();
};
}  // namespace rviz_vive_compositor_plugin

#endif
