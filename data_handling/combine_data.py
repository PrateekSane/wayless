import sys
import os
import rosbag

def combine_bags(output_bag_name, input_bag_files):
    if not input_bag_files:
        print("No bag files provided")
        sys.exit(1)

    # Check if all input files exist
    for bag_file in input_bag_files:
        if not os.path.exists(bag_file):
            print(f"Error: Bag file not found: {bag_file}")
            sys.exit(1)

    # Create output directory if it doesn't exist
    output_dir = os.path.dirname(output_bag_name)
    if output_dir and not os.path.exists(output_dir):
        print(f"Creating output directory: {output_dir}")
        os.makedirs(output_dir, exist_ok=True)

    print(f"Combining {len(input_bag_files)} bag files into {output_bag_name}")

    with rosbag.Bag(output_bag_name, 'w') as outbag:
        for bag_file in input_bag_files:
            print(f"Adding {bag_file}...")
            with rosbag.Bag(bag_file, 'r') as inbag:
                for topic, msg, t in inbag:
                    outbag.write(topic, msg, t)

    print(f"✅ Combined bag written to {output_bag_name}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 combine_data.py output.bag bag1.bag bag2.bag [bag3.bag ...]")
        print("Example: python3 combine_data.py combined.bag /path/to/bag1.bag /path/to/bag2.bag")
        sys.exit(1)

    output_bag = sys.argv[1]
    input_bag_files = sys.argv[2:]

    combine_bags(output_bag, input_bag_files)
