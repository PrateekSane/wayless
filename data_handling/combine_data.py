import sys
import os
import glob
import rosbag

def combine_bags(output_bag_name, input_dir):
    bag_files = sorted(glob.glob(os.path.join(input_dir, "*.bag")))
    if not bag_files:
        print(f"No .bag files found in directory: {input_dir}")
        sys.exit(1)

    print(f"Combining {len(bag_files)} bag files from {input_dir} into {output_bag_name}")

    with rosbag.Bag(output_bag_name, 'w') as outbag:
        for bag_file in bag_files:
            print(f"Adding {bag_file}...")
            with rosbag.Bag(bag_file, 'r') as inbag:
                for topic, msg, t in inbag:
                    outbag.write(topic, msg, t)

    print(f"✅ Combined bag written to {output_bag_name}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 combine_bags.py output.bag /path/to/bag_directory")
        sys.exit(1)

    output_bag = sys.argv[1]
    input_directory = sys.argv[2]

    combine_bags(output_bag, input_directory)
