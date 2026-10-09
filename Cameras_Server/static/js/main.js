// Configurazione
const UPDATE_INTERVAL = 2000; // Aggiorna stato e anteprime ogni 2 secondi
const cameras = ['rgb_left', 'rgb_right', 'event_left', 'event_right', 'lidar_livox', 'lidar_hesai'];

let currentSession = null;

// Inizializza la dashboard
document.addEventListener('DOMContentLoaded', function() {
    document.getElementById('server-ip').textContent = window.location.host;

    updateAllStatus();
    setInterval(updateAllStatus, UPDATE_INTERVAL);

    updateSystemTime();
    setInterval(updateSystemTime, 1000);

    cameras.forEach(refreshFrame);

    document.getElementById('refresh-all').addEventListener('click', () => cameras.forEach(refreshFrame));
    document.getElementById('start-all').addEventListener('click', startAll);
    document.getElementById('stop-all').addEventListener('click', stopAll);
    document.getElementById('clear-all').addEventListener('click', clearAll);
    document.getElementById('move-all').addEventListener('click', moveAll);
});

// Aggiorna l'orologio di sistema
function updateSystemTime() {
    document.getElementById('system-time').textContent = new Date().toLocaleTimeString('it-IT');
}

async function postJson(url) {
    const response = await fetch(url, { method: 'POST' });
    const data = await response.json();
    if (!response.ok) {
        throw new Error(data.error || `HTTP ${response.status}`);
    }
    return data;
}

// Aggiorna lo stato di tutte le telecamere (e le anteprime di quelle attive)
async function updateAllStatus() {
    try {
        const response = await fetch('/api/status');
        const status = await response.json();
        currentSession = status.session;
        document.getElementById('session-name').textContent = status.session || 'nessuna';
        const disk = document.getElementById('disk-free');
        disk.textContent = `${status.disk_free_gb} GB liberi` + (status.disk_stopped ? ' — REGISTRAZIONE FERMATA: DISCO PIENO' : '');
        disk.style.color = status.disk_stopped || status.disk_free_gb < 20 ? '#ef4444' : '';

        cameras.forEach(camera => {
            const s = status.sensors[camera];
            updateCameraStatus(camera, s);
            if (s.running) {
                refreshFrame(camera);
            }
        });
    } catch (error) {
        console.error('Errore nel recupero dello stato:', error);
        cameras.forEach(camera => {
            updateCameraStatus(camera, { active: false, last_update: null, fps: 0, error: 'server non raggiungibile' });
        });
    }
}

// Aggiorna lo stato di una singola telecamera
function updateCameraStatus(cameraName, status) {
    const statusIndicator = document.getElementById(`status-${cameraName}`);
    const fpsElement = document.getElementById(`fps-${cameraName}`);
    const timeElement = document.getElementById(`time-${cameraName}`);
    const errorElement = document.getElementById(`error-${cameraName}`);

    statusIndicator.classList.toggle('active', status.active);
    statusIndicator.classList.toggle('inactive', !status.active);

    fpsElement.textContent = `${status.fps || 0}${status.unit ? ' ' + status.unit : ''}`;
    timeElement.textContent = status.last_update || 'Mai';
    errorElement.textContent = status.error || status.warning || '';
}

// Aggiorna il frame di una telecamera
async function refreshFrame(cameraName) {
    const frameElement = document.getElementById(`frame-${cameraName}`);
    const loadingElement = document.getElementById(`loading-${cameraName}`);

    try {
        const response = await fetch(`/api/frame/${cameraName}`);
        if (!response.ok) {
            return; // Nessun frame ancora disponibile
        }
        const data = await response.json();
        frameElement.onload = function() {
            loadingElement.style.display = 'none';
            frameElement.classList.add('loaded');
        };
        frameElement.src = data.frame;
    } catch (error) {
        console.error(`Errore nel caricamento del frame per ${cameraName}:`, error);
    }
}

// Avvia una coppia stereo (rgb o event) o un lidar
async function startCameraType(cameraType) {
    try {
        await postJson(`/api/start/${cameraType}`);
    } catch (error) {
        alert(`Errore nell'avvio di ${cameraType}: ${error.message}`);
    }
    updateAllStatus();
}

// Ferma una coppia stereo (rgb o event) o un lidar
async function stopCameraType(cameraType) {
    try {
        await postJson(`/api/stop/${cameraType}`);
    } catch (error) {
        alert(`Errore nell'arresto di ${cameraType}: ${error.message}`);
    }
    updateAllStatus();
}

async function startAll() {
    try {
        await postJson('/api/start_all');
    } catch (error) {
        alert(`Errore nell'avvio: ${error.message}`);
    }
    updateAllStatus();
}

async function stopAll() {
    try {
        await postJson('/api/stop_all');
    } catch (error) {
        alert(`Errore nell'arresto: ${error.message}`);
    }
    updateAllStatus();
}

// Cancella la sessione corrente (tutti i dati registrati)
async function clearAll() {
    if (!currentSession) {
        alert('Nessuna sessione da cancellare');
        return;
    }
    const answer = prompt(`Cancellare DEFINITIVAMENTE la sessione ${currentSession}?\nScrivi "cancella" per confermare.`);
    if (answer !== 'cancella') {
        return;
    }
    try {
        await postJson('/api/clear_all');
        alert('Sessione cancellata');
    } catch (error) {
        alert(`Errore nella cancellazione: ${error.message}`);
    }
    updateAllStatus();
}

// Salva la sessione corrente in CVLab_live_imgs
async function moveAll() {
    if (!currentSession) {
        alert('Nessuna sessione da salvare');
        return;
    }
    if (!confirm(`Salvare la sessione ${currentSession} in CVLab_live_imgs?`)) {
        return;
    }
    try {
        const data = await postJson('/api/move_all');
        alert(`Sessione salvata in ${data.destination}`);
    } catch (error) {
        alert(`Errore nel salvataggio: ${error.message}`);
    }
    updateAllStatus();
}
