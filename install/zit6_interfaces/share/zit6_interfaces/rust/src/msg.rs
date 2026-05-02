#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to zit6_interfaces__msg__ZitSetpoint

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ZitSetpoint::default())
  }
}

impl rosidl_runtime_rs::Message for ZitSetpoint {
  type RmwMsg = super::msg::rmw::ZitSetpoint;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        control_key: msg.control_key,
        type_mask: msg.type_mask,
        x: msg.x,
        y: msg.y,
        z: msg.z,
        yaw: msg.yaw,
        seq: msg.seq,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      control_key: msg.control_key,
      type_mask: msg.type_mask,
      x: msg.x,
      y: msg.y,
      z: msg.z,
      yaw: msg.yaw,
      seq: msg.seq,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      control_key: msg.control_key,
      type_mask: msg.type_mask,
      x: msg.x,
      y: msg.y,
      z: msg.z,
      yaw: msg.yaw,
      seq: msg.seq,
    }
  }
}


// Corresponds to zit6_interfaces__msg__ZitStatus

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ZitStatus::default())
  }
}

impl rosidl_runtime_rs::Message for ZitStatus {
  type RmwMsg = super::msg::rmw::ZitStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        is_armed: msg.is_armed,
        control_level: msg.control_level,
        navigation_ready: msg.navigation_ready,
        forces: msg.forces,
        cycle_time_ms: msg.cycle_time_ms,
        battery_voltage: msg.battery_voltage,
        error_flags: msg.error_flags,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      is_armed: msg.is_armed,
      control_level: msg.control_level,
      navigation_ready: msg.navigation_ready,
        forces: msg.forces,
      cycle_time_ms: msg.cycle_time_ms,
      battery_voltage: msg.battery_voltage,
      error_flags: msg.error_flags,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      is_armed: msg.is_armed,
      control_level: msg.control_level,
      navigation_ready: msg.navigation_ready,
      forces: msg.forces,
      cycle_time_ms: msg.cycle_time_ms,
      battery_voltage: msg.battery_voltage,
      error_flags: msg.error_flags,
    }
  }
}


