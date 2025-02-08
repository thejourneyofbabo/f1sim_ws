use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::{Error, Result};
use nav_msgs::msg::Odometry;
use rclrs::{self, create_node, Context};
use sensor_msgs::msg::LaserScan;
use std::{
    env,
    sync::{Arc, Mutex},
};

struct WallFollowing {
    _laser_subscription: Arc<rclrs::Subscription<LaserScan>>,
    publication: Arc<rclrs::Publisher<AckermannDriveStamped>>,
    odom_msg: Arc<Mutex<Odometry>>,
}

impl WallFollowing {
    pub fn new(node: &rclrs::Node) -> Result<Self, rclrs::RclrsError> {}
    pub fn publish(&self) {}
}

fn main() -> Result<(), Error> {
    let context = Context::new(env::args())?;
    let node = create_node(&context, "wall_following")?;
    let subscriber_node_one = WallFollowing::new(&node)?;
    while context.ok() {
        subscriber_node_one.publish();
        let _ = rclrs::spin_once(node.clone(), Some(std::time::Duration::from_millis(500)));
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    Ok(())
}
