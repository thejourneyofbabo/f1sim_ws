use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::{Error, Result};
use rclrs::{self, create_node, Context};
use sensor_msgs::msg::LaserScan;
use std::env;

fn main() -> Result<(), Error> {
    let context = Context::new(env::args())?;
    let node = create_node(&context, "wall_following")
}
