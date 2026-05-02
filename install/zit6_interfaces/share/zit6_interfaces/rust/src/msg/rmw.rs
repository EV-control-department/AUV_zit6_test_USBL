#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "zit6_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__zit6_interfaces__msg__ZitSetpoint() -> *const std::ffi::c_void;
}

#[link(name = "zit6_interfaces__rosidl_generator_c")]
extern "C" {
    fn zit6_interfaces__msg__ZitSetpoint__init(msg: *mut ZitSetpoint) -> bool;
    fn zit6_interfaces__msg__ZitSetpoint__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ZitSetpoint>, size: usize) -> bool;
    fn zit6_interfaces__msg__ZitSetpoint__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ZitSetpoint>);
    fn zit6_interfaces__msg__ZitSetpoint__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ZitSetpoint>, out_seq: *mut rosidl_runtime_rs::Sequence<ZitSetpoint>) -> bool;
}

// Corresponds to zit6_interfaces__msg__ZitSetpoint
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ZitSetpoint {

    // This member is not documented.
    #[allow(missing_docs)]
    pub control_key: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub type_mask: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub x: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub z: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub seq: u32,

}



impl Default for ZitSetpoint {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !zit6_interfaces__msg__ZitSetpoint__init(&mut msg as *mut _) {
        panic!("Call to zit6_interfaces__msg__ZitSetpoint__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ZitSetpoint {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { zit6_interfaces__msg__ZitSetpoint__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { zit6_interfaces__msg__ZitSetpoint__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { zit6_interfaces__msg__ZitSetpoint__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ZitSetpoint {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ZitSetpoint where Self: Sized {
  const TYPE_NAME: &'static str = "zit6_interfaces/msg/ZitSetpoint";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__zit6_interfaces__msg__ZitSetpoint() }
  }
}


#[link(name = "zit6_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__zit6_interfaces__msg__ZitStatus() -> *const std::ffi::c_void;
}

#[link(name = "zit6_interfaces__rosidl_generator_c")]
extern "C" {
    fn zit6_interfaces__msg__ZitStatus__init(msg: *mut ZitStatus) -> bool;
    fn zit6_interfaces__msg__ZitStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ZitStatus>, size: usize) -> bool;
    fn zit6_interfaces__msg__ZitStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ZitStatus>);
    fn zit6_interfaces__msg__ZitStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ZitStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<ZitStatus>) -> bool;
}

// Corresponds to zit6_interfaces__msg__ZitStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ZitStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub is_armed: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub control_level: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub navigation_ready: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub forces: [f32; 4],


    // This member is not documented.
    #[allow(missing_docs)]
    pub cycle_time_ms: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub battery_voltage: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub error_flags: u32,

}



impl Default for ZitStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !zit6_interfaces__msg__ZitStatus__init(&mut msg as *mut _) {
        panic!("Call to zit6_interfaces__msg__ZitStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ZitStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { zit6_interfaces__msg__ZitStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { zit6_interfaces__msg__ZitStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { zit6_interfaces__msg__ZitStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ZitStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ZitStatus where Self: Sized {
  const TYPE_NAME: &'static str = "zit6_interfaces/msg/ZitStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__zit6_interfaces__msg__ZitStatus() }
  }
}


