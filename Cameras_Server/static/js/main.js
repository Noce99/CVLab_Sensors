// Configurazione
const UPDATE_INTERVAL = 3000; // Aggiorna ogni 3 secondi
const cameras = ['rgb_left', 'rgb_right', 'event_left', 'event_right', 'lidar_livox', 'lidar_hesai'];

// Inizializza la dashboard
document.addEventListener('DOMContentLoaded', function() {
    console.log('Dashboard inizializzata');
    
    // Mostra l'IP del server
    document.getElementById('server-ip').textContent = window.location.host;
    
    // Avvia l'aggiornamento automatico dello stato
    updateAllStatus();
    setInterval(updateAllStatus, UPDATE_INTERVAL);
    
    // Avvia l'aggiornamento dell'orologio
    updateSystemTime();
    setInterval(updateSystemTime, 1000);
    
    // Carica i frame iniziali
    cameras.forEach(camera => {
        refreshFrame(camera);
    });
    
    // Event listener per i pulsanti globali
    document.getElementById('refresh-all').addEventListener('click', function() {
        cameras.forEach(camera => {
            refreshFrame(camera);
        });
    });
    
    document.getElementById('start-all').addEventListener('click', startAll);
    document.getElementById('stop-all').addEventListener('click', stopAll);
    document.getElementById('clear-all').addEventListener('click', clearAll);
    document.getElementById('move-all').addEventListener('click', moveAll);
});

// Aggiorna l'orologio di sistema
function updateSystemTime() {
    const now = new Date();
    const timeString = now.toLocaleTimeString('it-IT');
    document.getElementById('system-time').textContent = timeString;
}

// Aggiorna lo stato di tutte le telecamere
async function updateAllStatus() {
    try {
        const response = await fetch('/api/status');
        const status = await response.json();
        
        cameras.forEach(camera => {
            updateCameraStatus(camera, status[camera]);
        });
    } catch (error) {
        console.error('Errore nel recupero dello stato:', error);
        cameras.forEach(camera => {
            updateCameraStatus(camera, { active: false, last_update: null, fps: 0 });
        });
    }
}

// Aggiorna lo stato di una singola telecamera
function updateCameraStatus(cameraName, status) {
    const statusIndicator = document.getElementById(`status-${cameraName}`);
    const fpsElement = document.getElementById(`fps-${cameraName}`);
    const timeElement = document.getElementById(`time-${cameraName}`);
    
    if (!statusIndicator || !fpsElement || !timeElement) {
        console.error(`Elementi non trovati per la camera: ${cameraName}`);
        return;
    }
    
    // Aggiorna indicatore di stato
    if (status.active) {
        statusIndicator.classList.add('active');
        statusIndicator.classList.remove('inactive');
    } else {
        statusIndicator.classList.remove('active');
        statusIndicator.classList.add('inactive');
    }
    
    // Aggiorna FPS
    fpsElement.textContent = status.fps || '0';
    
    // Aggiorna ultimo aggiornamento
    timeElement.textContent = status.last_update || 'Mai';
}

// Aggiorna il frame di una telecamera
async function refreshFrame(cameraName) {
    const frameElement = document.getElementById(`frame-${cameraName}`);
    const loadingElement = document.getElementById(`loading-${cameraName}`);
    
    if (!frameElement || !loadingElement) {
        console.error(`Elementi non trovati per la camera: ${cameraName}`);
        return;
    }
    
    // Mostra loading
    loadingElement.style.display = 'block';
    frameElement.classList.remove('loaded');
    
    try {
        const response = await fetch(`/api/frame/${cameraName}`);
        
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        
        const data = await response.json();
        
        // Aggiorna l'immagine
        frameElement.src = data.frame;
        frameElement.onload = function() {
            loadingElement.style.display = 'none';
            frameElement.classList.add('loaded');
        };
        
    } catch (error) {
        console.error(`Errore nel caricamento del frame per ${cameraName}:`, error);
        loadingElement.textContent = 'Errore nel caricamento';
        loadingElement.style.color = '#ef4444';
        
        // Reset dopo 3 secondi
        setTimeout(() => {
            loadingElement.textContent = 'Caricamento...';
            loadingElement.style.color = '#666';
        }, 3000);
    }
}

// Avvia una coppia stereo (rgb o event)
async function startCameraType(cameraType) {
    try {
        const response = await fetch(`/api/start/${cameraType}`);
        const data = await response.json();
        
        console.log(`Camera type ${cameraType} avviata:`, data);
        
        // Mostra notifica
        showNotification(data.message || `Camera ${cameraType} avviata`, 'success');
        
        // Aggiorna lo stato
        updateAllStatus();
        
    } catch (error) {
        console.error(`Errore nell'avvio della camera ${cameraType}:`, error);
        showNotification(`Errore nell'avvio della camera ${cameraType}`, 'error');
    }
}

// Ferma una coppia stereo (rgb o event)
async function stopCameraType(cameraType) {
    try {
        const response = await fetch(`/api/stop/${cameraType}`);
        const data = await response.json();
        
        console.log(`Camera type ${cameraType} fermata:`, data);
        
        // Mostra notifica
        showNotification(data.message || `Camera ${cameraType} fermata`, 'info');
        
        // Aggiorna lo stato
        updateAllStatus();
        
    } catch (error) {
        console.error(`Errore nell'arresto della camera ${cameraType}:`, error);
        showNotification(`Errore nell'arresto della camera ${cameraType}`, 'error');
    }
}

// Avvia tutte le telecamere
async function startAll() {
    try {
        const response = await fetch('/api/start_all');
        const data = await response.json();
        
        console.log('Tutte le camere avviate:', data);
        showNotification('Avvio di tutte le telecamere...', 'success');
        
        // Aggiorna lo stato
        setTimeout(updateAllStatus, 500);
        
    } catch (error) {
        console.error('Errore nell\'avvio di tutte le camere:', error);
        showNotification('Errore nell\'avvio', 'error');
    }
}

// Ferma tutte le telecamere
async function stopAll() {
    try {
        const response = await fetch('/api/stop_all');
        const data = await response.json();
        
        console.log('Tutte le camere fermate:', data);
        showNotification('Arresto di tutte le telecamere...', 'info');
        
        // Aggiorna lo stato
        setTimeout(updateAllStatus, 500);
        
    } catch (error) {
        console.error('Errore nell\'arresto di tutte le camere:', error);
        showNotification('Errore nell\'arresto', 'error');
    }
}

// Avvia una telecamera
async function startCamera(cameraName) {
    try {
        const response = await fetch(`/api/start/${cameraName}`);
        const data = await response.json();
        
        console.log(`Camera ${cameraName} avviata:`, data);
        
        // Mostra notifica
        showNotification(`Camera ${cameraName} avviata`, 'success');
        
        // Aggiorna lo stato
        updateAllStatus();
        
    } catch (error) {
        console.error(`Errore nell'avvio della camera ${cameraName}:`, error);
        showNotification(`Errore nell'avvio della camera ${cameraName}`, 'error');
    }
}

// Ferma una telecamera
async function stopCamera(cameraName) {
    try {
        const response = await fetch(`/api/stop/${cameraName}`);
        const data = await response.json();
        
        console.log(`Camera ${cameraName} fermata:`, data);
        
        // Mostra notifica
        showNotification(`Camera ${cameraName} fermata`, 'info');
        
        // Aggiorna lo stato
        updateAllStatus();
        
    } catch (error) {
        console.error(`Errore nell'arresto della camera ${cameraName}:`, error);
        showNotification(`Errore nell'arresto della camera ${cameraName}`, 'error');
    }
}


// Cancella dati dal disco
async function clearAll() {
    try {
        const response = await fetch('/api/clear_all');
        const data = await response.json();
        
        console.log('Cancellati tutti i dati salvati:', data);
        showNotification('Cancello tutti i dati presi...', 'info');
        
        // Aggiorna lo stato
        updateAllStatus();
        
    } catch (error) {
        console.error('Errore nella cancellazione delle immagini salvate:', error);
        showNotification('Errore nella cancellazione immagini', 'error');
    }
}

// Sposta Img salvate su altra cartella ripulendo live_imgs
async function moveAll() {
    try {
        const response = await fetch('/api/move_all');
        const data = await response.json();
        
        console.log('Sposto tutte le live_imgs in ~/Documents/CVLab_live_imgs:', data);
        showNotification('Sposto tutte le live_imgs in ~/Documents/CVLab_live_imgs:', 'info');
        
        // Aggiorna lo stato
        updateAllStatus;
        
    } catch (error) {
        console.error('Errore nello spostamento delle immagini:', error);
        showNotification('Errore nello spostamento delle immagini', 'error');
    }
}

// Mostra una notifica (implementazione semplice)
function showNotification(message, type) {
    // Per ora solo console.log, puoi implementare toast notifications
    console.log(`[${type.toUpperCase()}] ${message}`);
    

}

// Auto-refresh dei frame (opzionale, commentato per default)
// Decommenta se vuoi un refresh automatico continuo
/*
setInterval(() => {
    cameras.forEach(camera => {
        refreshFrame(camera);
    });
}, 5000); // Ogni 5 secondi
*/
