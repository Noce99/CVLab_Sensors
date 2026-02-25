#!/usr/bin/env python3
"""
Script di test per verificare le telecamere RGB Basler
Esegui questo prima di avviare il server per verificare che l'hardware funzioni
"""

import sys

try:
    from pypylon import pylon
    print("✅ pypylon importato correttamente")
except ImportError:
    print("❌ pypylon non disponibile")
    print("   Installa con: pip install pypylon")
    sys.exit(1)

LEFT_SERIAL = "40663113"
RIGHT_SERIAL = "40665216"

def test_cameras():
    print("\n" + "="*60)
    print("TEST TELECAMERE RGB BASLER")
    print("="*60)
    
    # Cerca dispositivi
    print("\n1. Ricerca dispositivi Pylon...")
    tlFactory = pylon.TlFactory.GetInstance()
    devices = tlFactory.EnumerateDevices()
    
    if not devices:
        print("❌ Nessun dispositivo Pylon trovato!")
        print("   Verifica:")
        print("   - Le telecamere sono connesse via USB?")
        print("   - I driver Pylon sono installati?")
        return False
    
    print(f"✅ Trovati {len(devices)} dispositivi")
    
    # Mostra tutti i dispositivi
    print("\n2. Dispositivi disponibili:")
    for i, device in enumerate(devices):
        print(f"   [{i}] {device.GetFriendlyName()}")
        print(f"       Serial: {device.GetSerialNumber()}")
        print(f"       Model:  {device.GetModelName()}")
    
    # Cerca le telecamere specifiche
    print("\n3. Ricerca telecamere configurate...")
    left_found = False
    right_found = False
    
    for device in devices:
        serial = device.GetSerialNumber()
        if serial == LEFT_SERIAL:
            print(f"   ✅ LEFT camera trovata: {serial}")
            left_found = True
        elif serial == RIGHT_SERIAL:
            print(f"   ✅ RIGHT camera trovata: {serial}")
            right_found = True
    
    if not left_found:
        print(f"   ❌ LEFT camera ({LEFT_SERIAL}) non trovata")
    if not right_found:
        print(f"   ❌ RIGHT camera ({RIGHT_SERIAL}) non trovata")
    
    if not (left_found and right_found):
        print("\n⚠️  Modifica i serial number in camera_server.py")
        return False
    
    # Test acquisizione
    print("\n4. Test acquisizione (5 frame)...")
    try:
        cameras = pylon.InstantCameraArray(2)
        
        for device in devices:
            if device.GetSerialNumber() == LEFT_SERIAL:
                cameras[0].Attach(tlFactory.CreateDevice(device))
                cameras[0].Open()
            elif device.GetSerialNumber() == RIGHT_SERIAL:
                cameras[1].Attach(tlFactory.CreateDevice(device))
                cameras[1].Open()
        
        cameras.StartGrabbing()
        
        for i in range(5):
            grabResult = cameras.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            
            if grabResult.GrabSucceeded():
                camera_index = grabResult.GetCameraContext()
                camera_name = "LEFT" if camera_index == 0 else "RIGHT"
                timestamp = grabResult.GetTimeStamp()
                shape = grabResult.GetArray().shape
                
                print(f"   ✅ Frame {i+1}: {camera_name} - {shape} - ts: {timestamp}")
            
            grabResult.Release()
        
        cameras.StopGrabbing()
        cameras.Close()
        
        print("✅ Test acquisizione completato")
        
    except Exception as e:
        print(f"❌ Errore durante acquisizione: {e}")
        return False
    
    print("\n" + "="*60)
    print("✅ TUTTI I TEST SUPERATI!")
    print("="*60)
    print("\nPuoi avviare il server con: python camera_server.py")
    return True

if __name__ == "__main__":
    success = test_cameras()
    sys.exit(0 if success else 1)
