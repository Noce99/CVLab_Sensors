#!/bin/bash
# Start the Camera Monitoring Server (works from any directory)
cd "$(dirname "$0")" || exit 1

echo "Avvio Camera Monitoring Server"
echo ""
echo "IP disponibili per la connessione:"
hostname -I | tr ' ' '\n' | grep -v '^$'
echo ""
echo "Connettiti dal cellulare a: http://<IP>:5000"
echo "Oppure da questo computer a: http://localhost:5000"
echo ""
echo "Premi Ctrl+C per fermare il server"
echo ""

../venv_3_12/bin/python camera_server.py
