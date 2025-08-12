const api = {
    list: '/check_data/list_files',
    file_info: '/check_data/file_info',
    frame: '/check_data/get_frame',
    joints: '/check_data/joints'
};

let current = {
    folder: '',
    filename: '',
    info: null,
    rgbDatasets: [],
    jointDatasets: [],
    players: [
        { dataset: null, count: 0, idx: 0, playing: false },
        { dataset: null, count: 0, idx: 0, playing: false },
        { dataset: null, count: 0, idx: 0, playing: false }
    ]
};

document.getElementById('btnList').addEventListener('click', async () => {
    const folder = document.getElementById('folder').value.trim();
    if (!folder) {
        alert('请输入文件夹路径');
        return;
    }
    try {
        const resp = await fetch(api.list, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ folder })
        });
        const data = await resp.json();
        if (data.error) {
            alert('错误: ' + data.error);
            return;
        }
        const fileList = document.getElementById('fileList');
        fileList.innerHTML = '';
        data.files.forEach(filename => {
            const li = document.createElement('li');
            li.textContent = filename;
            li.addEventListener('click', () => {
                document.querySelectorAll('#fileList li').forEach(el => el.classList.remove('selected'));
                li.classList.add('selected');
                current.filename = li.textContent;   // 这里赋值选中文件名
                current.folder = document.getElementById('folder').value.trim();  // 也同步folder
                document.getElementById('btnLoad').disabled = false;
            });

            fileList.appendChild(li);
        });
    } catch (e) {
        alert('请求失败: ' + e.message);
    }
});

document.getElementById('btnLoad').onclick = async () => {
    if (!current.folder || !current.filename) return alert('先选文件');
    const r = await fetch(api.file_info, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ folder: current.folder, filename: current.filename })
    });
    const info = await r.json();
    if (info.error) return alert(info.error);
    current.info = info;
    renderInfo(info);
    populateDatasets(info);
    await loadJointsAndPlot();
};

function renderInfo(info) {
    const d = document.getElementById('fileInfo');
    d.innerHTML = '';
    const table = document.createElement('table');
    table.className = 'meta';
    const tbody = document.createElement('tbody');
    for (const name of Object.keys(info.datasets || {})) {
        const r = document.createElement('tr');
        const td1 = document.createElement('td'); td1.textContent = name;
        const td2 = document.createElement('td'); td2.textContent = JSON.stringify(info.datasets[name].shape);
        r.appendChild(td1); r.appendChild(td2);
        tbody.appendChild(r);
    }
    table.appendChild(tbody);
    d.appendChild(table);
}
async function loadJointsAndPlot() {
    console.log('加载关节数据...');
    const datasets = ['action', 'observations/qpos'];
    const calls = datasets.map(d => fetch(api.joints, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ folder: current.folder, filename: current.filename, dataset: d })
    }).then(r => r.json()));

    const results = await Promise.all(calls);

    console.log('关节数据返回', results);

    let labels = [];
    let series = [];

    results.forEach((res, idx) => {
        if (res.error) {
            alert(res.error);
            return;
        }
        const arr = res.data; // 可能是二维数组 M x D
        // flatten 取均值作为单值
        const vals = arr.map(s => Array.isArray(s) ? mean(s) : s);
        series.push({ label: datasets[idx], data: vals });
        if (!labels.length) labels = vals.map((_, i) => i);
    });

    console.log('绘制数据:', { labels, series });
    renderChart(labels, series);
}

// 载入文件信息时，设置视频播放器，使用所有检测到的 rgbDatasets
function populateDatasets(info) {
    const rgbs = info.detected_rgbs || [];
    for (let i = 0; i < 3; i++) {
        const ds = rgbs[i] || null;
        const p = current.players[i];
        p.dataset = ds;
        p.count = info.rgb_counts && info.rgb_counts[ds] ? info.rgb_counts[ds] : 0;
        updatePlayerUI(i);
    }
}
function mean(a) { return a.reduce((s, x) => s + (typeof x === 'number' ? x : 0), 0) / (a.length || 1); }

let chart = null;
function renderChart(labels, series) {
    const ctx = document.getElementById('chartJoints').getContext('2d');

    if (window.jointsChart) {
        window.jointsChart.destroy();
    }

    window.jointsChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: labels,
            datasets: series.map((s, i) => ({
                label: s.label,
                data: s.data,
                borderColor: i === 0 ? 'rgba(63, 81, 181, 1)' : 'rgba(255, 99, 132, 1)',
                backgroundColor: i === 0 ? 'rgba(63, 81, 181, 0.3)' : 'rgba(255, 99, 132, 0.3)',
                fill: false,
                tension: 0.2,
                pointRadius: 0,
            })),
        },
        options: {
            responsive: true,
            animation: false,
            interaction: {
                mode: 'nearest',
                intersect: false,
            },
            scales: {
                x: {
                    display: true,
                    title: {
                        display: true,
                        text: 'Index',
                    },
                },
                y: {
                    display: true,
                    title: {
                        display: true,
                        text: 'Value',
                    },
                },
            },
        },
    });
}
async function fetchFrame(dataset, idx) {
    const url = `${api.frame}?folder=${encodeURIComponent(current.folder)}&filename=${encodeURIComponent(current.filename)}&dataset=${encodeURIComponent(dataset)}&idx=${idx}`;
    const r = await fetch(url);
    if (!r.ok) return null;
    const blob = await r.blob();
    const img = new Image();
    return new Promise(resolve => {
        img.onload = () => { URL.revokeObjectURL(img.src); resolve(img); };
        img.onerror = () => resolve(null);
        img.src = URL.createObjectURL(blob);
    });
}


function updatePlayerUI(i) {
    const p = current.players[i];
    document.getElementById('p' + i + 'name').textContent = p.dataset || '-';
    const prog = document.querySelector(`input.progress[data-idx="${i}"]`);
    prog.max = p.count > 0 ? String(p.count - 1) : '0';
    prog.value = p.idx;
    document.querySelector(`.pos[data-idx="${i}"]`).textContent = `${p.idx} / ${p.count}`;
}

document.querySelectorAll('.play').forEach(btn => {
    btn.addEventListener('click', async (ev) => {
        const idx = Number(ev.currentTarget.dataset.idx);
        const p = current.players[idx];
        if (!p.dataset) return alert('未选择 dataset');
        p.playing = !p.playing;
        ev.currentTarget.textContent = p.playing ? 'Pause' : 'Play';
        if (p.playing) playLoop(idx);
    });
});

document.querySelectorAll('input.progress').forEach(inp => {
    inp.addEventListener('input', (ev) => {
        const idx = Number(ev.target.dataset.idx);
        current.players[idx].idx = Number(ev.target.value);
        updatePlayerUI(idx);
        drawFrameToCanvas(idx);
    });
});

async function playLoop(i) {
    const p = current.players[i];
    while (p.playing) {
        // draw current frame
        await drawFrameToCanvas(i);
        // advance
        p.idx++;
        if (p.idx >= p.count) { p.idx = 0; } // loop
        updatePlayerUI(i);
        // simple timing (~30fps)
        await new Promise(r => setTimeout(r, 33));
    }
}

async function drawFrameToCanvas(i) {
    const p = current.players[i];
    const cv = document.getElementById('cv' + i);
    const ctx = cv.getContext('2d');
    if (!p.dataset || p.count <= 0) {
        ctx.clearRect(0, 0, cv.width, cv.height);
        return;
    }
    const img = await fetchFrame(p.dataset, p.idx);
    if (!img) return;
    // fit into canvas
    const cw = cv.width, ch = cv.height;
    // scale to fit
    const ratio = Math.min(cw / img.width, ch / img.height);
    const w = img.width * ratio, h = img.height * ratio;
    ctx.clearRect(0, 0, cw, ch);
    ctx.drawImage(img, (cw - w) / 2, (ch - h) / 2, w, h);
}