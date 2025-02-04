use ackermann_msgs::msg::AckermannDriveStamped; // AckermannDriveStamped로 변경
use anyhow::{Error, Result};
use rclrs::{self, Context};
use std::{env, thread, time::Duration};

fn main() -> Result<(), Error> {
    println!("This is test move node");
    let context = Context::new(env::args())?;
    let node = rclrs::create_node(&context, "test_drive")?;

    let publisher =
        node.create_publisher::<AckermannDriveStamped>("/drive", rclrs::QOS_PROFILE_DEFAULT)?;

    let mut msg = AckermannDriveStamped::default();
    for _ in 0..100 {
        msg.drive.speed = 1.0;
        thread::sleep(Duration::from_millis(10));
        let _ = publisher.publish(&msg);
    }

    for _ in 0..100 {
        msg.drive.speed = 0.0;
        thread::sleep(Duration::from_millis(10));
        let _ = publisher.publish(&msg);
    }

    Ok(())
}
