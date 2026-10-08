/* eslint-disable */
// These files were generated using "ros-typescript-generator"
export interface EufsMsgsBoundingBox {
  color: string;
  probability: number;
  type: number;
  xmin: number;
  ymin: number;
  xmax: number;
  ymax: number;
}

export enum EufsMsgsBoundingBoxConst {
  PIXEL = 0,
  PERCENTAGE = 1,
}

export interface EufsMsgsBoundingBoxes {
  header: StdMsgsHeader;
  image_header: StdMsgsHeader;
  bounding_boxes: EufsMsgsBoundingBox[];
}

export interface EufsMsgsCanState {
  as_state: number;
  ami_state: number;
}

export enum EufsMsgsCanStateConst {
  AS_OFF = 0,
  AS_READY = 1,
  AS_DRIVING = 2,
  AS_EMERGENCY_BRAKE = 3,
  AS_FINISHED = 4,
  AMI_NOT_SELECTED = 10,
  AMI_ACCELERATION = 11,
  AMI_SKIDPAD = 12,
  AMI_AUTOCROSS = 13,
  AMI_TRACK_DRIVE = 14,
  AMI_AUTONOMOUS_DEMO = 15,
  AMI_ADS_INSPECTION = 16,
  AMI_ADS_EBS = 17,
  AMI_DDT_INSPECTION_A = 18,
  AMI_DDT_INSPECTION_B = 19,
  AMI_JOYSTICK = 20,
  AMI_MANUAL = 21,
}

export interface EufsMsgsCarForces {
  front_left_lateral: number;
  front_left_longitudinal: number;
  front_left_vertical: number;
  front_right_lateral: number;
  front_right_longitudinal: number;
  front_right_vertical: number;
  rear_left_lateral: number;
  rear_left_longitudinal: number;
  rear_left_vertical: number;
  rear_right_lateral: number;
  rear_right_longitudinal: number;
  rear_right_vertical: number;
}

export interface EufsMsgsCarState {
  header: StdMsgsHeader;
  child_frame_id: string;
  pose: GeometryMsgsPoseWithCovariance;
  twist: GeometryMsgsTwistWithCovariance;
  linear_acceleration: GeometryMsgsVector3;
  linear_acceleration_covariance: number[];
  slip_angle: number;
  state_of_charge: number;
}

export interface EufsMsgsChassisCommand {
  header: StdMsgsHeader;
  sender: string;
  throttle: number;
  steering: number;
  front_brake: number;
}

export interface EufsMsgsChassisState {
  header: StdMsgsHeader;
  throttle_relay_enabled: boolean;
  autonomous_enabled: boolean;
  runstop_motion_enabled: boolean;
  steering_commander: string;
  steering: number;
  throttle_commander: string;
  throttle: number;
  front_brake_commander: string;
  front_brake: number;
}

export interface EufsMsgsCheckForObjectsActionGoal {
  id: number;
  image: SensorMsgsImage;
}

export interface EufsMsgsCheckForObjectsActionResult {
  id: number;
  bounding_boxes: EufsMsgsBoundingBoxes;
}

export interface EufsMsgsConeArray {
  header: StdMsgsHeader;
  blue_cones: GeometryMsgsPoint[];
  yellow_cones: GeometryMsgsPoint[];
  orange_cones: GeometryMsgsPoint[];
  big_orange_cones: GeometryMsgsPoint[];
  unknown_color_cones: GeometryMsgsPoint[];
}

export interface EufsMsgsConeArrayWithCovariance {
  header: StdMsgsHeader;
  blue_cones: EufsMsgsConeWithCovariance[];
  yellow_cones: EufsMsgsConeWithCovariance[];
  orange_cones: EufsMsgsConeWithCovariance[];
  big_orange_cones: EufsMsgsConeWithCovariance[];
  unknown_color_cones: EufsMsgsConeWithCovariance[];
}

export interface EufsMsgsConeArrayWithCovariancePlus {
  header: StdMsgsHeader;
  cones: EufsMsgsConeWithCovariancePlus[];
}

export interface EufsMsgsConeAssociation {
  first: EufsMsgsConeWithCovariance;
  second: EufsMsgsConeWithCovariance;
}

export interface EufsMsgsConeAssociationArray {
  type: string;
  threshold: number;
  matched: EufsMsgsConeAssociation[];
  unmatched: EufsMsgsConeWithCovariance[];
}

export interface EufsMsgsConeAssociationArrayArrayStamped {
  header: StdMsgsHeader;
  associations: EufsMsgsConeAssociationArray[];
}

export interface EufsMsgsConeAssociationArrayStamped {
  header: StdMsgsHeader;
  associations: EufsMsgsConeAssociationArray;
}

export interface EufsMsgsConeWithCovariance {
  point: GeometryMsgsPoint;
  covariance: number[];
}

export interface EufsMsgsConeWithCovariancePlus {
  id: number;
  blue_prob: number;
  yellow_prob: number;
  orange_prob: number;
  big_orange_prob: number;
  unknown_prob: number;
  point: GeometryMsgsPoint;
  covariance: number[];
}

export interface EufsMsgsCostmap {
  x_bounds_min: number;
  x_bounds_max: number;
  y_bounds_min: number;
  y_bounds_max: number;
  pixels_per_meter: number;
  channel0: number[];
  channel1: number[];
  channel2: number[];
  channel3: number[];
}

export interface EufsMsgsCpuStatus {
  status: number;
  total: number;
  usr: number;
  nice: number;
  sys: number;
  idle: number;
}

export enum EufsMsgsCpuStatusConst {
  OK = 0,
  HIGH_LOAD = 1,
  VERY_HIGH_LOAD = 2,
  STALE = 3,
}

export interface EufsMsgsCpuUsage {
  stamp: { sec: number, nanosec: number };
  all: EufsMsgsCpuStatus;
  cpus: EufsMsgsCpuStatus[];
}

export interface EufsMsgsEkfErr {
  header: StdMsgsHeader;
  gps_x_vel_err: number;
  gps_y_vel_err: number;
  imu_x_acc_err: number;
  imu_y_acc_err: number;
  imu_yaw_err: number;
  ekf_x_vel_var: number;
  ekf_y_vel_var: number;
  ekf_x_acc_var: number;
  ekf_y_acc_var: number;
  ekf_yaw_var: number;
}

export interface EufsMsgsEkfState {
  gps_received: boolean;
  imu_received: boolean;
  wheel_odom_received: boolean;
  ekf_odom_received: boolean;
  ekf_accel_received: boolean;
  currently_over_covariance_limit: boolean;
  consecutive_turns_over_covariance_limit: boolean;
  recommends_failure: boolean;
}

export interface EufsMsgsFullState {
  header: StdMsgsHeader;
  x_pos: number;
  y_pos: number;
  yaw: number;
  roll: number;
  u_x: number;
  u_y: number;
  yaw_mder: number;
  front_throttle: number;
  rear_throttle: number;
  steering: number;
}

export interface EufsMsgsGetMapResponse {
  map_path: string[];
}

export interface EufsMsgsHeartbeat {
  id: number;
  data: number;
}

export interface EufsMsgsIntegrationErr {
  header: StdMsgsHeader;
  position_err: number;
  orientation_err: number;
  linear_vel_err: number;
  angular_vel_err: number;
}

export interface EufsMsgsLapStats {
  header: StdMsgsHeader;
  lap_number: number;
  lap_time: number;
  avg_speed: number;
  max_speed: number;
  speed_var: number;
  max_slip: number;
  max_lateral_accel: number;
  normalized_deviation_mse: number;
  deviation_var: number;
  max_deviation: number;
}

export interface EufsMsgsMpcState {
  exitflag: number;
  iterations: number;
  solve_time: number;
  cost: number;
}

export interface EufsMsgsNodeState {
  id: number;
  name: string;
  exp_heartbeat: number;
  last_heartbeat: { sec: number, nanosec: number };
  severity: number;
  online: boolean;
}

export interface EufsMsgsNodeStateArray {
  header: StdMsgsHeader;
  states: EufsMsgsNodeState[];
}

export interface EufsMsgsOss {
  header: StdMsgsHeader;
  velocity: number;
  v_x: number;
  v_y: number;
  yaw_rate: number;
}

export interface EufsMsgsParticle {
  pose: GeometryMsgsPose;
  map: EufsMsgsConeArrayWithCovariance;
  weight: number;
}

export interface EufsMsgsParticleSlam {
  best_particle_idx: number;
  particles: EufsMsgsParticle[];
}

export interface EufsMsgsParticleSlamStamped {
  header: StdMsgsHeader;
  state: EufsMsgsParticleSlam;
}

export interface EufsMsgsParticleStamped {
  header: StdMsgsHeader;
  particle: EufsMsgsParticle;
}

export interface EufsMsgsPathIntegralParams {
  hz: number;
  num_timesteps: number;
  num_iters: number;
  gamma: number;
  init_steering: number;
  init_throttle: number;
  steering_var: number;
  throttle_var: number;
  max_throttle: number;
  speed_coefficient: number;
  track_coefficient: number;
  max_slip_angle: number;
  track_slop: number;
  crash_coeff: number;
  map_path: string;
  desired_speed: number;
}

export interface EufsMsgsPathIntegralStats {
  header: StdMsgsHeader;
  tag: string;
  params: EufsMsgsPathIntegralParams;
  stats: EufsMsgsLapStats;
}

export interface EufsMsgsPathIntegralStatus {
  header: StdMsgsHeader;
  info: string;
  status: number;
}

export interface EufsMsgsPathIntegralTiming {
  header: StdMsgsHeader;
  average_time_between_poses: number;
  average_optimization_cycle_time: number;
  average_sleep_time: number;
}

export interface EufsMsgsPlanningMode {
  mode: number;
}

export enum EufsMsgsPlanningModeConst {
  LOCAL = 0,
  GLOBAL = 1,
}

export interface EufsMsgsPointArray {
  points: GeometryMsgsPoint[];
}

export interface EufsMsgsPointArrayStamped {
  header: StdMsgsHeader;
  points: GeometryMsgsPoint[];
}

export interface EufsMsgsPurePursuitCheckpoint {
  position: GeometryMsgsPoint;
  max_speed: number;
  max_lateral_acceleration: number;
}

export interface EufsMsgsPurePursuitCheckpointArrayStamped {
  header: StdMsgsHeader;
  checkpoints: EufsMsgsPurePursuitCheckpoint[];
}

export interface EufsMsgsRecordStart {
  request: EufsMsgsRecordStartRequest;
  response: EufsMsgsRecordStartResponse;
}

export interface EufsMsgsRecordStartRequest {
  pack_number: number;
}

export interface EufsMsgsRecordStartResponse {
  start_recording: boolean;
}

export interface EufsMsgsRecordStopResponse {
  stop_recording: boolean;
}

export interface EufsMsgsRegister {
  request: EufsMsgsRegisterRequest;
  response: EufsMsgsRegisterResponse;
}

export interface EufsMsgsRegisterRequest {
  node_name: string;
  severity: number;
}

export interface EufsMsgsRegisterResponse {
  id: number;
}

export interface EufsMsgsRunstop {
  header: StdMsgsHeader;
  sender: string;
  motion_enabled: boolean;
}

export interface EufsMsgsSetCanState {
  request: EufsMsgsSetCanStateRequest;
  response: EufsMsgsSetCanStateResponse;
}

export interface EufsMsgsSetCanStateRequest {
  ami_state: number;
  as_state: number;
}

export interface EufsMsgsSetCanStateResponse {
  success: boolean;
  message: string;
}

export interface EufsMsgsSetMission {
  request: EufsMsgsSetMissionRequest;
  response: EufsMsgsSetMissionResponse;
}

export interface EufsMsgsSetMissionRequest {
  mission: number;
}

export enum EufsMsgsSetMissionRequestConst {
  NOT_SELECTED = 0,
  ACCELERATION = 1,
  SKIDPAD = 2,
  AUTOCROSS = 3,
  TRACK_DRIVE = 4,
  MANUAL_DRIVING = 5,
  ADS_EBS_TEST = 6,
  ADS_INSPECTION = 7,
  DDT_INSPECTION_A = 8,
  DDT_INSPECTION_B = 9,
  DDT_AUTONOMOUS_DEMO = 10,
}

export interface EufsMsgsSetMissionResponse {
  success: boolean;
  message: string;
}

export interface EufsMsgsSetString {
  request: EufsMsgsSetStringRequest;
  response: EufsMsgsSetStringResponse;
}

export interface EufsMsgsSetStringRequest {
  data: string;
}

export interface EufsMsgsSetStringResponse {
  success: boolean;
  message: string;
}

export interface EufsMsgsSetTrack {
  request: EufsMsgsSetTrackRequest;
  response: EufsMsgsSetTrackResponse;
}

export interface EufsMsgsSetTrackRequest {
  data: EufsMsgsConeArrayWithCovariance;
}

export interface EufsMsgsSetTrackResponse {
  success: boolean;
  message: string;
}

export interface EufsMsgsSlamErr {
  header: StdMsgsHeader;
  x_err: number;
  y_err: number;
  z_err: number;
  x_orient_err: number;
  y_orient_err: number;
  z_orient_err: number;
  w_orient_err: number;
  map_similarity: number;
}

export interface EufsMsgsSlamState {
  loop_closed: boolean;
  laps: number;
  status: string;
  state: number;
}

export enum EufsMsgsSlamStateConst {
  NO_INPUTS = 0,
  MAPPING = 1,
  LOCALISING = 2,
  TIMEOUT = 3,
  RECOMMENDS_FAILURE = 4,
}

export interface EufsMsgsStateMachineState {
  state: number;
  state_str: string;
}

export interface EufsMsgsStereoImage {
  header: StdMsgsHeader;
  color: SensorMsgsImage;
  depth: SensorMsgsImage;
}

export interface EufsMsgsSystemStatus {
  header: StdMsgsHeader;
  topic_statuses: EufsMsgsTopicStatus[];
}

export interface EufsMsgsTopicStatus {
  topic: string;
  description: string;
  group: string;
  trigger_ebs: boolean;
  log_level: string;
  status: number;
}

export enum EufsMsgsTopicStatusConst {
  OFF = 0,
  PUBLISHING = 1,
  TIMEOUT_EXCEEDED = 2,
}

export interface EufsMsgsVehicleCommands {
  handshake: number;
  ebs: number;
  direction: number;
  mission_status: number;
  braking: number;
  torque: number;
  steering: number;
  rpm: number;
}

export interface EufsMsgsVehicleCommandsStamped {
  header: StdMsgsHeader;
  commands: EufsMsgsVehicleCommands;
}

export interface EufsMsgsWaypoint {
  position: GeometryMsgsPoint;
  speed: number;
  suggested_steering: number;
}

export interface EufsMsgsWaypointArrayStamped {
  header: StdMsgsHeader;
  waypoints: EufsMsgsWaypoint[];
}

export interface EufsMsgsWheelOdometryErr {
  header: StdMsgsHeader;
  position_err: number;
  orientation_err: number;
  linear_vel_err: number;
  angular_vel_err: number;
  forward_vel_err: number;
}

export interface EufsMsgsWheelSpeeds {
  steering: number;
  lf_speed: number;
  rf_speed: number;
  lb_speed: number;
  rb_speed: number;
}

export interface EufsMsgsWheelSpeedsStamped {
  header: StdMsgsHeader;
  speeds: EufsMsgsWheelSpeeds;
}

export interface GeometryMsgsAccel {
  linear: GeometryMsgsVector3;
  angular: GeometryMsgsVector3;
}

export interface GeometryMsgsAccelStamped {
  header: StdMsgsHeader;
  accel: GeometryMsgsAccel;
}

export interface GeometryMsgsAccelWithCovariance {
  accel: GeometryMsgsAccel;
  covariance: number[];
}

export interface GeometryMsgsAccelWithCovarianceStamped {
  header: StdMsgsHeader;
  accel: GeometryMsgsAccelWithCovariance;
}

export interface GeometryMsgsInertia {
  m: number;
  com: GeometryMsgsVector3;
  ixx: number;
  ixy: number;
  ixz: number;
  iyy: number;
  iyz: number;
  izz: number;
}

export interface GeometryMsgsInertiaStamped {
  header: StdMsgsHeader;
  inertia: GeometryMsgsInertia;
}

export interface GeometryMsgsPoint {
  x: number;
  y: number;
  z: number;
}

export interface GeometryMsgsPoint32 {
  x: number;
  y: number;
  z: number;
}

export interface GeometryMsgsPointStamped {
  header: StdMsgsHeader;
  point: GeometryMsgsPoint;
}

export interface GeometryMsgsPolygon {
  points: GeometryMsgsPoint32[];
}

export interface GeometryMsgsPolygonStamped {
  header: StdMsgsHeader;
  polygon: GeometryMsgsPolygon;
}

export interface GeometryMsgsPose {
  position: GeometryMsgsPoint;
  orientation: GeometryMsgsQuaternion;
}

export interface GeometryMsgsPose2D {
  x: number;
  y: number;
  theta: number;
}

export interface GeometryMsgsPoseArray {
  header: StdMsgsHeader;
  poses: GeometryMsgsPose[];
}

export interface GeometryMsgsPoseStamped {
  header: StdMsgsHeader;
  pose: GeometryMsgsPose;
}

export interface GeometryMsgsPoseWithCovariance {
  pose: GeometryMsgsPose;
  covariance: number[];
}

export interface GeometryMsgsPoseWithCovarianceStamped {
  header: StdMsgsHeader;
  pose: GeometryMsgsPoseWithCovariance;
}

export interface GeometryMsgsQuaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface GeometryMsgsQuaternionStamped {
  header: StdMsgsHeader;
  quaternion: GeometryMsgsQuaternion;
}

export interface GeometryMsgsTransform {
  translation: GeometryMsgsVector3;
  rotation: GeometryMsgsQuaternion;
}

export interface GeometryMsgsTransformStamped {
  header: StdMsgsHeader;
  child_frame_id: string;
  transform: GeometryMsgsTransform;
}

export interface GeometryMsgsTwist {
  linear: GeometryMsgsVector3;
  angular: GeometryMsgsVector3;
}

export interface GeometryMsgsTwistStamped {
  header: StdMsgsHeader;
  twist: GeometryMsgsTwist;
}

export interface GeometryMsgsTwistWithCovariance {
  twist: GeometryMsgsTwist;
  covariance: number[];
}

export interface GeometryMsgsTwistWithCovarianceStamped {
  header: StdMsgsHeader;
  twist: GeometryMsgsTwistWithCovariance;
}

export interface GeometryMsgsVector3 {
  x: number;
  y: number;
  z: number;
}

export interface GeometryMsgsVector3Stamped {
  header: StdMsgsHeader;
  vector: GeometryMsgsVector3;
}

export interface GeometryMsgsWrench {
  force: GeometryMsgsVector3;
  torque: GeometryMsgsVector3;
}

export interface GeometryMsgsWrenchStamped {
  header: StdMsgsHeader;
  wrench: GeometryMsgsWrench;
}

export interface SensorMsgsBatteryState {
  header: StdMsgsHeader;
  voltage: number;
  temperature: number;
  current: number;
  charge: number;
  capacity: number;
  design_capacity: number;
  percentage: number;
  power_supply_status: number;
  power_supply_health: number;
  power_supply_technology: number;
  present: boolean;
  cell_voltage: number[];
  cell_temperature: number[];
  location: string;
  serial_number: string;
}

export enum SensorMsgsBatteryStateConst {
  POWER_SUPPLY_STATUS_UNKNOWN = 0,
  POWER_SUPPLY_STATUS_CHARGING = 1,
  POWER_SUPPLY_STATUS_DISCHARGING = 2,
  POWER_SUPPLY_STATUS_NOT_CHARGING = 3,
  POWER_SUPPLY_STATUS_FULL = 4,
  POWER_SUPPLY_HEALTH_UNKNOWN = 0,
  POWER_SUPPLY_HEALTH_GOOD = 1,
  POWER_SUPPLY_HEALTH_OVERHEAT = 2,
  POWER_SUPPLY_HEALTH_DEAD = 3,
  POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4,
  POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5,
  POWER_SUPPLY_HEALTH_COLD = 6,
  POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7,
  POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8,
  POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0,
  POWER_SUPPLY_TECHNOLOGY_NIMH = 1,
  POWER_SUPPLY_TECHNOLOGY_LION = 2,
  POWER_SUPPLY_TECHNOLOGY_LIPO = 3,
  POWER_SUPPLY_TECHNOLOGY_LIFE = 4,
  POWER_SUPPLY_TECHNOLOGY_NICD = 5,
  POWER_SUPPLY_TECHNOLOGY_LIMN = 6,
}

export interface SensorMsgsCameraInfo {
  header: StdMsgsHeader;
  height: number;
  width: number;
  distortion_model: string;
  d: number[];
  k: number[];
  r: number[];
  p: number[];
  binning_x: number;
  binning_y: number;
  roi: SensorMsgsRegionOfInterest;
}

export interface SensorMsgsChannelFloat32 {
  name: string;
  values: number[];
}

export interface SensorMsgsCompressedImage {
  header: StdMsgsHeader;
  format: string;
  data: number[];
}

export interface SensorMsgsFluidPressure {
  header: StdMsgsHeader;
  fluid_pressure: number;
  variance: number;
}

export interface SensorMsgsIlluminance {
  header: StdMsgsHeader;
  illuminance: number;
  variance: number;
}

export interface SensorMsgsImage {
  header: StdMsgsHeader;
  height: number;
  width: number;
  encoding: string;
  is_bigendian: number;
  step: number;
  data: number[];
}

export interface SensorMsgsImu {
  header: StdMsgsHeader;
  orientation: GeometryMsgsQuaternion;
  orientation_covariance: number[];
  angular_velocity: GeometryMsgsVector3;
  angular_velocity_covariance: number[];
  linear_acceleration: GeometryMsgsVector3;
  linear_acceleration_covariance: number[];
}

export interface SensorMsgsJointState {
  header: StdMsgsHeader;
  name: string[];
  position: number[];
  velocity: number[];
  effort: number[];
}

export interface SensorMsgsJoy {
  header: StdMsgsHeader;
  axes: number[];
  buttons: number[];
}

export interface SensorMsgsJoyFeedback {
  type: number;
  id: number;
  intensity: number;
}

export enum SensorMsgsJoyFeedbackConst {
  TYPE_LED = 0,
  TYPE_RUMBLE = 1,
  TYPE_BUZZER = 2,
}

export interface SensorMsgsJoyFeedbackArray {
  array: SensorMsgsJoyFeedback[];
}

export interface SensorMsgsLaserEcho {
  echoes: number[];
}

export interface SensorMsgsLaserScan {
  header: StdMsgsHeader;
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  time_increment: number;
  scan_time: number;
  range_min: number;
  range_max: number;
  ranges: number[];
  intensities: number[];
}

export interface SensorMsgsMagneticField {
  header: StdMsgsHeader;
  magnetic_field: GeometryMsgsVector3;
  magnetic_field_covariance: number[];
}

export interface SensorMsgsMultiDofJointState {
  header: StdMsgsHeader;
  joint_names: string[];
  transforms: GeometryMsgsTransform[];
  twist: GeometryMsgsTwist[];
  wrench: GeometryMsgsWrench[];
}

export interface SensorMsgsMultiEchoLaserScan {
  header: StdMsgsHeader;
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  time_increment: number;
  scan_time: number;
  range_min: number;
  range_max: number;
  ranges: SensorMsgsLaserEcho[];
  intensities: SensorMsgsLaserEcho[];
}

export interface SensorMsgsNavSatFix {
  header: StdMsgsHeader;
  status: SensorMsgsNavSatStatus;
  latitude: number;
  longitude: number;
  altitude: number;
  position_covariance: number[];
  position_covariance_type: number;
}

export enum SensorMsgsNavSatFixConst {
  COVARIANCE_TYPE_UNKNOWN = 0,
  COVARIANCE_TYPE_APPROXIMATED = 1,
  COVARIANCE_TYPE_DIAGONAL_KNOWN = 2,
  COVARIANCE_TYPE_KNOWN = 3,
}

export interface SensorMsgsNavSatStatus {
  status: number;
  service: number;
}

export enum SensorMsgsNavSatStatusConst {
  STATUS_NO_FIX = -1,
  STATUS_FIX = 0,
  STATUS_SBAS_FIX = 1,
  STATUS_GBAS_FIX = 2,
  SERVICE_GPS = 1,
  SERVICE_GLONASS = 2,
  SERVICE_COMPASS = 4,
  SERVICE_GALILEO = 8,
}

export interface SensorMsgsPointCloud {
  header: StdMsgsHeader;
  points: GeometryMsgsPoint32[];
  channels: SensorMsgsChannelFloat32[];
}

export interface SensorMsgsPointCloud2 {
  header: StdMsgsHeader;
  height: number;
  width: number;
  fields: SensorMsgsPointField[];
  is_bigendian: boolean;
  point_step: number;
  row_step: number;
  data: number[];
  is_dense: boolean;
}

export interface SensorMsgsPointField {
  name: string;
  offset: number;
  datatype: number;
  count: number;
}

export enum SensorMsgsPointFieldConst {
  INT8 = 1,
  UINT8 = 2,
  INT16 = 3,
  UINT16 = 4,
  INT32 = 5,
  UINT32 = 6,
  FLOAT32 = 7,
  FLOAT64 = 8,
}

export interface SensorMsgsRange {
  header: StdMsgsHeader;
  radiation_type: number;
  field_of_view: number;
  min_range: number;
  max_range: number;
  range: number;
}

export enum SensorMsgsRangeConst {
  ULTRASOUND = 0,
  INFRARED = 1,
}

export interface SensorMsgsRegionOfInterest {
  x_offset: number;
  y_offset: number;
  height: number;
  width: number;
  do_rectify: boolean;
}

export interface SensorMsgsRelativeHumidity {
  header: StdMsgsHeader;
  relative_humidity: number;
  variance: number;
}

export interface SensorMsgsSetCameraInfo {
  request: SensorMsgsSetCameraInfoRequest;
  response: SensorMsgsSetCameraInfoResponse;
}

export interface SensorMsgsSetCameraInfoRequest {
  camera_info: SensorMsgsCameraInfo;
}

export interface SensorMsgsSetCameraInfoResponse {
  success: boolean;
  status_message: string;
}

export interface SensorMsgsTemperature {
  header: StdMsgsHeader;
  temperature: number;
  variance: number;
}

export interface SensorMsgsTimeReference {
  header: StdMsgsHeader;
  time_ref: { sec: number, nanosec: number };
  source: string;
}

export interface StdMsgsBool {
  data: boolean;
}

export interface StdMsgsByte {
  data: number;
}

export interface StdMsgsByteMultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsChar {
  data: number;
}

export interface StdMsgsColorRgba {
  r: number;
  g: number;
  b: number;
  a: number;
}

export interface StdMsgsFloat32 {
  data: number;
}

export interface StdMsgsFloat32MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsFloat64 {
  data: number;
}

export interface StdMsgsFloat64MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsHeader {
  stamp: { sec: number, nanosec: number };
  frame_id: string;
}

export interface StdMsgsInt16 {
  data: number;
}

export interface StdMsgsInt16MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsInt32 {
  data: number;
}

export interface StdMsgsInt32MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsInt64 {
  data: number;
}

export interface StdMsgsInt64MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsInt8 {
  data: number;
}

export interface StdMsgsInt8MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsMultiArrayDimension {
  label: string;
  size: number;
  stride: number;
}

export interface StdMsgsMultiArrayLayout {
  dim: StdMsgsMultiArrayDimension[];
  data_offset: number;
}

export interface StdMsgsString {
  data: string;
}

export interface StdMsgsUInt16 {
  data: number;
}

export interface StdMsgsUInt16MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsUInt32 {
  data: number;
}

export interface StdMsgsUInt32MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsUInt64 {
  data: number;
}

export interface StdMsgsUInt64MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface StdMsgsUInt8 {
  data: number;
}

export interface StdMsgsUInt8MultiArray {
  layout: StdMsgsMultiArrayLayout;
  data: number[];
}

export interface VisualizationMsgsGetInteractiveMarkers {
  response: VisualizationMsgsGetInteractiveMarkersResponse;
}

export interface VisualizationMsgsGetInteractiveMarkersResponse {
  sequence_number: number;
  markers: VisualizationMsgsInteractiveMarker[];
}

export interface VisualizationMsgsImageMarker {
  header: StdMsgsHeader;
  ns: string;
  id: number;
  type: number;
  action: number;
  position: GeometryMsgsPoint;
  scale: number;
  outline_color: StdMsgsColorRgba;
  filled: number;
  fill_color: StdMsgsColorRgba;
  lifetime: { sec: number, nanosec: number };
  points: GeometryMsgsPoint[];
  outline_colors: StdMsgsColorRgba[];
}

export enum VisualizationMsgsImageMarkerConst {
  CIRCLE = 0,
  LINE_STRIP = 1,
  LINE_LIST = 2,
  POLYGON = 3,
  POINTS = 4,
  ADD = 0,
  REMOVE = 1,
}

export interface VisualizationMsgsInteractiveMarker {
  header: StdMsgsHeader;
  pose: GeometryMsgsPose;
  name: string;
  description: string;
  scale: number;
  menu_entries: VisualizationMsgsMenuEntry[];
  controls: VisualizationMsgsInteractiveMarkerControl[];
}

export interface VisualizationMsgsInteractiveMarkerControl {
  name: string;
  orientation: GeometryMsgsQuaternion;
  orientation_mode: number;
  interaction_mode: number;
  always_visible: boolean;
  markers: VisualizationMsgsMarker[];
  independent_marker_orientation: boolean;
  description: string;
}

export enum VisualizationMsgsInteractiveMarkerControlConst {
  INHERIT = 0,
  FIXED = 1,
  VIEW_FACING = 2,
  NONE = 0,
  MENU = 1,
  BUTTON = 2,
  MOVE_AXIS = 3,
  MOVE_PLANE = 4,
  ROTATE_AXIS = 5,
  MOVE_ROTATE = 6,
  MOVE_3D = 7,
  ROTATE_3D = 8,
  MOVE_ROTATE_3D = 9,
}

export interface VisualizationMsgsInteractiveMarkerFeedback {
  header: StdMsgsHeader;
  client_id: string;
  marker_name: string;
  control_name: string;
  event_type: number;
  pose: GeometryMsgsPose;
  menu_entry_id: number;
  mouse_point: GeometryMsgsPoint;
  mouse_point_valid: boolean;
}

export enum VisualizationMsgsInteractiveMarkerFeedbackConst {
  KEEP_ALIVE = 0,
  POSE_UPDATE = 1,
  MENU_SELECT = 2,
  BUTTON_CLICK = 3,
  MOUSE_DOWN = 4,
  MOUSE_UP = 5,
}

export interface VisualizationMsgsInteractiveMarkerInit {
  server_id: string;
  seq_num: number;
  markers: VisualizationMsgsInteractiveMarker[];
}

export interface VisualizationMsgsInteractiveMarkerPose {
  header: StdMsgsHeader;
  pose: GeometryMsgsPose;
  name: string;
}

export interface VisualizationMsgsInteractiveMarkerUpdate {
  server_id: string;
  seq_num: number;
  type: number;
  markers: VisualizationMsgsInteractiveMarker[];
  poses: VisualizationMsgsInteractiveMarkerPose[];
  erases: string[];
}

export enum VisualizationMsgsInteractiveMarkerUpdateConst {
  KEEP_ALIVE = 0,
  UPDATE = 1,
}

export interface VisualizationMsgsMarker {
  header: StdMsgsHeader;
  ns: string;
  id: number;
  type: number;
  action: number;
  pose: GeometryMsgsPose;
  scale: GeometryMsgsVector3;
  color: StdMsgsColorRgba;
  lifetime: { sec: number, nanosec: number };
  frame_locked: boolean;
  points: GeometryMsgsPoint[];
  colors: StdMsgsColorRgba[];
  texture_resource: string;
  texture: SensorMsgsCompressedImage;
  uv_coordinates: VisualizationMsgsUvCoordinate[];
  text: string;
  mesh_resource: string;
  mesh_file: VisualizationMsgsMeshFile;
  mesh_use_embedded_materials: boolean;
}

export enum VisualizationMsgsMarkerConst {
  ARROW = 0,
  CUBE = 1,
  SPHERE = 2,
  CYLINDER = 3,
  LINE_STRIP = 4,
  LINE_LIST = 5,
  CUBE_LIST = 6,
  SPHERE_LIST = 7,
  POINTS = 8,
  TEXT_VIEW_FACING = 9,
  MESH_RESOURCE = 10,
  TRIANGLE_LIST = 11,
  ADD = 0,
  MODIFY = 0,
  DELETE = 2,
  DELETEALL = 3,
}

export interface VisualizationMsgsMarkerArray {
  markers: VisualizationMsgsMarker[];
}

export interface VisualizationMsgsMenuEntry {
  id: number;
  parent_id: number;
  title: string;
  command: string;
  command_type: number;
}

export enum VisualizationMsgsMenuEntryConst {
  FEEDBACK = 0,
  ROSRUN = 1,
  ROSLAUNCH = 2,
}

export interface VisualizationMsgsMeshFile {
  filename: string;
  data: number[];
}

export interface VisualizationMsgsUvCoordinate {
  u: number;
  v: number;
}