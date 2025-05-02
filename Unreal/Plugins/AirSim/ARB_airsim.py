import airsim
import numpy as np

# 클라이언트 연결
client = airsim.CarClient()
client.confirmConnection()

try:
    # Lidar 데이터 가져오기
    lidar_data = client.getLidarData(lidar_name="Lidar1", vehicle_name="Car1")
    
    # 데이터가 있는지 확인
    if len(lidar_data.point_cloud) > 0:
        # point cloud 데이터 출력
        print("Raw point cloud data:")
        print(lidar_data.point_cloud)
        
        # points 배열로 변환
        points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
        print(f"\nPoint cloud shape: {points.shape}")
        print("\nFirst few points:")
        print(points[:5])  # 처음 5개 포인트만 출력
        
    else:
        print("No point cloud data received")
        
except Exception as e:
    print(f"Error getting lidar data: {str(e)}") 