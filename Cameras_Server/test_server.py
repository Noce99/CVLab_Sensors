#!/usr/bin/env python3
"""
Script di test per verificare il server
Esegui questo mentre il server è in esecuzione
"""

import requests
import json
import time

SERVER_URL = "http://localhost:5000"

def test_status_endpoint():
    """Test endpoint /api/status"""
    print("\n=== Test /api/status ===")
    try:
        response = requests.get(f"{SERVER_URL}/api/status")
        print(f"Status code: {response.status_code}")
        if response.status_code == 200:
            data = response.json()
            print(json.dumps(data, indent=2))
            return True
    except Exception as e:
        print(f"Errore: {e}")
        return False

def test_frame_endpoint(camera_name):
    """Test endpoint /api/frame/<camera>"""
    print(f"\n=== Test /api/frame/{camera_name} ===")
    try:
        response = requests.get(f"{SERVER_URL}/api/frame/{camera_name}")
        print(f"Status code: {response.status_code}")
        if response.status_code == 200:
            data = response.json()
            print(f"Frame ricevuto: {len(data['frame'])} bytes")
            print(f"Timestamp: {data['timestamp']}")
            return True
    except Exception as e:
        print(f"Errore: {e}")
        return False

def test_start_stop(camera_name):
    """Test endpoint start/stop"""
    print(f"\n=== Test start/stop per {camera_name} ===")
    try:
        # Stop
        response = requests.get(f"{SERVER_URL}/api/stop/{camera_name}")
        print(f"Stop - Status code: {response.status_code}")
        time.sleep(1)
        
        # Start
        response = requests.get(f"{SERVER_URL}/api/start/{camera_name}")
        print(f"Start - Status code: {response.status_code}")
        return True
    except Exception as e:
        print(f"Errore: {e}")
        return False

def main():
    print("🧪 Test del Camera Monitoring Server")
    print(f"Server: {SERVER_URL}")
    print("\nAssicurati che il server sia in esecuzione!")
    input("Premi INVIO per iniziare i test...")
    
    cameras = ['rgb', 'event', 'lidar_livox', 'lidar_hesai']
    
    # Test status
    if not test_status_endpoint():
        print("\n❌ Server non raggiungibile!")
        return
    
    # Test frame per ogni camera
    print("\n" + "="*50)
    for camera in cameras:
        test_frame_endpoint(camera)
        time.sleep(0.5)
    
    # Test start/stop
    print("\n" + "="*50)
    test_start_stop('rgb')
    
    print("\n" + "="*50)
    print("✅ Test completati!")
    print("\nApri il browser a: http://localhost:5000")

if __name__ == "__main__":
    main()
