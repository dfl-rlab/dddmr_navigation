import torch
import cv2

def main():
    print("Testing PyTorch CUDA availability...")
    # Assert PyTorch can access CUDA
    assert torch.cuda.is_available() == True, "PyTorch CUDA is NOT enabled."
    print(f"PyTorch CUDA is enabled. Version: {torch.version.cuda}")

    print("\nTesting OpenCV CUDA availability...")
    # Assert OpenCV was built with CUDA support
    build_info = cv2.getBuildInformation()
    assert "CUDA" in build_info, "OpenCV is NOT compiled with CUDA."
    
    # Assert OpenCV can access CUDA devices
    assert hasattr(cv2, 'cuda'), "cv2.cuda module not found."
    count = cv2.cuda.getCudaEnabledDeviceCount()
    assert count > 0, "OpenCV CUDA is compiled, but no CUDA devices were found."
    print(f"OpenCV CUDA is enabled. Devices found: {count}")

    print("\nAll CUDA assertions passed successfully!")

if __name__ == '__main__':
    main()
