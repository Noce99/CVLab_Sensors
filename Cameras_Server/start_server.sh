#!/bin/bash

echo "Avvio Camera Monitoring Server"
echo ""

# # Controlla se Python è installato
# if ! command -v python3 &> /dev/null; then
#     echo "Python3 non trovato. Installalo prima di continuare."
#     exit 1
# fi

# # Crea virtual environment se non esiste
# if [ ! -d "venv" ]; then
#     echo "Creazione virtual environment..."
#     python3 -m venv venv
# fi

# # Attiva virtual environment
# echo "🔧 Attivazione virtual environment..."
# source venv/bin/activate

# # Controlla se le dipendenze sono installate
# echo "cd Controllo dipendenze..."
# python -c "import flask" 2>/dev/null
# if [ $? -ne 0 ]; then
#     echo "Dipendenze mancanti. Installazione in corso..."
#     pip install -r requirements.txt
# fi

# Trova l'IP locale
echo ""
echo "IP disponibili per la connessione:"
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    hostname -I | tr ' ' '\n' | grep -v '^$'
elif [[ "$OSTYPE" == "darwin"* ]]; then
    ifconfig | grep "inet " | grep -v 127.0.0.1 | awk '{print $2}'
fi

echo ""
echo "Connettiti dal cellulare a: http://<IP>:5000"
echo "Oppure da questo computer a: http://localhost:5000"
echo ""
echo "Premi Ctrl+C per fermare il server"
echo ""

# Avvia il server
../venv_3_12/bin/python camera_server.py
