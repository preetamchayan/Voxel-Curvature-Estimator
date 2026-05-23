from pathlib import Path

def read_triplets(filename):
    """Read triplets of integers from a file into a set of tuples."""
    triplets = set()
    with open(filename, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 3:
                triplets.add(tuple(map(int, parts)))
    return triplets

def main(file1, file2):
    # Read triplets from both files
    triplets1 = read_triplets(file1)
    triplets2 = read_triplets(file2)

    # Find unique triplets
    only_in_file1 = triplets1 - triplets2
    only_in_file2 = triplets2 - triplets1

    # Print results
    for triplet in sorted(only_in_file1):
        print(f"{triplet} is present only in {file1}")
    for triplet in sorted(only_in_file2):
        print(f"{triplet} is present only in {file2}")

if __name__ == "__main__":
    # Example usage: replace with your actual file names
    root = Path(__file__).parent.parent
    output_dir = root / "output"
    main(output_dir / "voxel_export_log-PARALLEL-VULKAN.txt", output_dir / "voxel_export_log-SERIAL.txt")
