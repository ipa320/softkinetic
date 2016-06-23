^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package softkinetic_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.2 (2016-06-23)
------------------

0.6.1 (2016-06-21)
------------------
* 0.6.1
* added CHANGELOG
* consistent version nummber
* Revert "Point cloud reformation to kinect-like "
* demo launch file modified according to frame_id change and standardize aspect ratio
* compile dummy node if the depthsense sdk is not found (used for travis compilation)
* remove non-rgb pointcloud
* point cloud frame_id fixed to rgb_optical_frame
* Merge pull request `#55 <https://github.com/ipa320/softkinetic/issues/55>`_ from knorth55/serial-cfg-fix
  fix dynamic_reconfigure config
* add time stamp in PointCloud ROS msg
* fix dynamic_reconfigure config
* Merge pull request `#54 <https://github.com/ipa320/softkinetic/issues/54>`_ from knorth55/add-keep-organized
  add setKeepOrganized to get organized output
* Merge pull request `#53 <https://github.com/ipa320/softkinetic/issues/53>`_ from knorth55/filter-default-cfg-fixed
  Fix use_filter default value True -> False
* Merge pull request `#52 <https://github.com/ipa320/softkinetic/issues/52>`_ from knorth55/image-format-fixed
  Fix image format WXGA -> WXGA_H
* add use_serial in cfg
* add use_serial and serial rosparam
* add setKeepOrganized to get organized output
* fix use_filter default value True -> False
* fix image format WXGA -> WXGA_H
* Remove useles variables
* Get some performance improvement by not processing 3D points for
  saturated pixels (as they are anyway all-zero points).
  Also enforce calling any other filter before radious outlier removal so
  we get rid of the all-zero points and it doesn't take ages (I have
  tweaked default parameters accordingly).
* Fix fustrum filter camera correction acording to the projection in
  indigo (that turned to be wrong! see issue `#46 <https://github.com/ipa320/softkinetic/issues/46>`_).
  Also, fustrum filter only works with fov < 180, so I set max values to
  180 (and also set more realistic bounds for close/far planes)
* Code simplification
* Merge branch 'pal-robotics-forks-hydro_dev' into merge_pal
  Intermedate step before merging into indigo.
  I retain from PAL PR the dynamir reconfigure, the yaml configuration and
  the two additional filters (though frustum culling is not working!)
* disable not used outputs on depth node (doesn't improve performance,
  anyway)
* Replace ProjectionHelper by the UVmap; seems to be a bit faster and for sure makes the code a bit cleaner
* Fix the collision box orientation. Also set realistic values for inertia
  , calculated with the solid cuboid formula from here:
  http://en.wikipedia.org/wiki/List_of_moments_of_inertia
* Convert softkinetic vertices into a kinect-like coordinates pointcloud.
  Required by 83b8a8f2cfdcdc0441e4b433ad86cb16f378e54b
* Replicate kinect tf scheme.
* Merge branch 'hydro_dev' of https://github.com/pal-robotics/softkinetic into hydro_dev
* fix variable name
* fixes minor warnings and cosmetics
* adds range_max param
  - reprojects x, y, z = range_max when no reading
  - uses i=j=0 instead of i=j=1 (last 2 rows were discarded)
* Merge remote-tracking branch 'upstream/hydro_dev' into hydro_dev
  Conflicts:
  softkinetic_camera/CMakeLists.txt
* adds frustrum culling filter
* Issue `#25 <https://github.com/ipa320/softkinetic/issues/25>`_: Color image is wrongly formed because the step field is not set
* Pull request `#30 <https://github.com/ipa320/softkinetic/issues/30>`_: a complete driver revision of the driver
  Also puts indigo branch up to date with hydro, so PR `#33 <https://github.com/ipa320/softkinetic/issues/33>`_ is unnecessary
* dynamic reconfigure parameter not taking effect
  confidence_threshold update through dynamic reconfigure was not being applied
* Merge branch 'hydro_dev' of https://github.com/pal-robotics/softkinetic into hydro_dev
* fix reading of /camera_link parameter
  a private NodeHandle is used to retrieve "camera_link" parameter. However, the parameter name was set to "/camera_link" so that it was being looked for in the root namespace and therefore, not found.
* fix setupCameraInfo to use cam_info of the method's arg
* Replicate PR `#32 <https://github.com/ipa320/softkinetic/issues/32>`_ to speed up both color and depth images reading
* use memcpy to copy color image instead of nested for-loops
* renames voxel_grid_size and min_neighbours params
* base_rgbd_camera_optical_frame on static_transform
* drops outdated opencv2 dependency for indigo
* adds dynamic reconfigure server
* updates indentation
* adds passthrough filter params
* adds passthrough filter
* Merge pull request `#5 <https://github.com/ipa320/softkinetic/issues/5>`_ from efernandez/link
  Sets camera_link to optical frame
* sets optical frame (not camera link directly)
* use C++ casting
* We get wrong image_color and image_mono when color_compression is set to
  YUY2. This problem is also present in hydro_devel. Fixed reading color
  map as a YUV image and converting into BGR, with very low performance
  penalization.
  I have also wrote some additional comments.
* fix camera frame name
* Rename parameter minNeighboursInRadius as min_neighbours to respect
  parameters naming conventions
* Add launch files specific for each softkinetic camera
* Add enabling/disabling params for depth and color
* add launch file for the ds325 model
  Specific parameters set for this model
* Add camera views
* Separated calibration files for depth and color cameras.
* ROS-formated
* Big rebouild to improve speed, fix depth camera info, change topics
  names, etc.
* Merge pull request `#27 <https://github.com/ipa320/softkinetic/issues/27>`_ from pal-robotics/hydro_dev
  Add params to enable/disable color and depth streams
* Publish mono and depth images and rgb/depth camera info.
  Some other small fixes
* add launch file for the ds325 model
  Specific parameters set for this model
* fix identation
* add enabling/disabling params for depth and color
* Issue `#17 <https://github.com/ipa320/softkinetic/issues/17>`_: Add color information to published PointCloud2 messages
* Add all recent merges on upstream
* sets base_rgdb_camera_link
* adds install rule for launch files
* Add camera descrption xacro file
* Add camera descrption xacro file
* Issue `#17 <https://github.com/ipa320/softkinetic/issues/17>`_: Add color information to published PointCloud2 messages
* remove unused variables
* comments filters on launch file
* demo launch includes the other launch file now
* add depth image viewer in rviz configuration
* add ROS param to enable point cloud downsampling
  the param use_voxel_grid_filter can be used now to enable the existing downsampling filter in the code
* add depth image publisher
  and launch file for the DS311 camera without launching rviz
* add ROS params to configure depth and color
  Params to configure depth mode, resolution and rate
  Params to configure color compression, resolution and rate
  Add specific launch file for the DS311 model as the default configuration works properly for the DS325 model
* Remove opencv2 dependency according to Indigo migration rules
* build: avoid hardcoding DepthSense SDK location.
  Introduce pkg local Find script, and adjust include and linking
  statements to use it.
* Remove multiple pcl::PointCloud <-> sensor_msgs::PointCloud2 conversions
  on filtering (~20% faster)
  Also adds a voxel grid side parameter to do more or less radical
  downsamples
* Missing install tags
* added missing arguments to softkinetic_camera_demo.launch
* added missing arguments
* included missing header references
* merge
* Created single camera demo launch file
* implemented downsampling routine in order to speed up radius based filtering
* added point cloud filter (radius based)
* modified ROS debug messages in src-file and in launch file
* launch file adjustments
* minor adjustments
* Update package.xml
* To suppress the error, header should use the pcl_conversions in hydro according to http://wiki.ros.org/hydro/Migration#PCL
* cout error in case no cameras where found
* parameter adjustment
* README update
* removed hydro compilation error, converted point cloud output to sensor_msgs::PointCloud2 and updated README.md
* added parameter confidence_threshold for DepthNode configuration via parameter server and updated default launch file correspondingly
* remove eigen
* move to separate repo
* Contributors: Benjamin Maidel, Dave Coleman, Felipe Garcia Lopez, Florian Weisshardt, Jordi Pages, Jorge Santos, Marcus Liebhardt, Matthias Gruhler, Shingo Kitagawa, Yutaka Kondo, aginika, corot, enriquefernandez, flg, gavanderhoorn, ipa-cob4-1, ipa-fmw, ipa-jenkins, ipa-nhg
