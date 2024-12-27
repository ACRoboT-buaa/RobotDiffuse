/* globals */
import * as THREE from 'three';
import { registerDragEvents } from './dragAndDrop.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader.js';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import URDFManipulator from '../../src/urdf-manipulator-element.js';
	customElements.define('urdf-viewer', URDFManipulator);

	// declare these globally for the sake of the example.
	// Hack to make the build work with webpack for now.
	// TODO: Remove this once modules or parcel is being used
	const viewer = document.querySelector('urdf-viewer');

	const startToggle = document.getElementById('start');
	const upSelect = document.getElementById('up-select');
	const sliderList = document.querySelector('#angle');
	const obs1 = document.querySelector('#obs1');
	const obs2 = document.querySelector('#obs2');
	const controlsel = document.getElementById('controls');
	const controlsToggle = document.getElementById('toggle-controls');
	const stopToggle = document.getElementById('stop');
	const DEG2RAD = Math.PI / 180;
	const RAD2DEG = 1 / DEG2RAD;
	const button = document.getElementById('button');
	const button2 = document.getElementById('button2');
	const cancel = document.getElementById('cancel');
	let sliders = {};

	// Global Functions
	const setColor = color => {

	    document.body.style.backgroundColor = color;
	    viewer.highlightColor = '#' + (new Color(0xffffff)).lerp(new Color(color), 0.35).getHexString();

	};
	window.current=0;
	window.pos0=[-1.57,-1.23,1.68,-1.38,-1.31,2.85,1.68];
	window.pos1=[0.21,-1.21,-1.78,-2.45,1.73,2.62,-1.52];
	window.stop=0;
	function animate(data){
		window.data=JSON.parse(data);
		var pos = 0;
		var id = setInterval(frame, 10);
		function frame() {
			if(window.stop==1){
				clearInterval(id);
				let event = new Event("click");
				stopToggle.dispatchEvent(event);
				return;
			}
			if(pos==0){
				window.current=2;
				pos++;
			}
			else if (pos == 254) {
				pos=0;
			} else {
				for(var i=1;i<=7;i++){
					viewer.setAngle(`panda_joint${ i }`, window.data[pos][i-1]);
				}
				pos++;
			}
		}
		  

	}
	function httpGetAsync(theUrl, data, callback)
	{
		var xmlHttp = new XMLHttpRequest();
		window.stop=0;
		xmlHttp.onreadystatechange = function() { 
			if (xmlHttp.readyState == 4 && xmlHttp.status == 200)
				callback(xmlHttp.responseText);
		}
		xmlHttp.open("POST", theUrl, true); // true for asynchronous 
		xmlHttp.setRequestHeader("Content-Type", "application/json");
		

		xmlHttp.send(JSON.stringify(data));
	}
	button.addEventListener('click',()=>{
		var req=window.pos0.concat(window.pos1).concat(window.obs);
		httpGetAsync("http://127.0.0.1:5000",req,animate);
	})
	button2.addEventListener('click',()=>{
		var req=window.pos0.concat(window.pos1).concat(window.obs);
		httpGetAsync("http://127.0.0.1:5000/rrt",req,animate);
	})
	cancel.addEventListener('click',()=>{
		window.stop=1;
	})




	// Events
	// toggle checkbox
	startToggle.addEventListener('click', () => {
		if(!startToggle.classList.contains('checked')){
			startToggle.classList.add('checked');
			stopToggle.classList.remove('checked');
			window.current=0;
			for(var i=1;i<=7;i++){
				viewer.setAngle(`panda_joint${ i }`, window.pos0[i-1]);
			}
		}
	});
	stopToggle.addEventListener('click', () => {
		if(!stopToggle.classList.contains('checked')){
			stopToggle.classList.add('checked');
			startToggle.classList.remove('checked');
			window.current=1;
			for(var i=1;i<=7;i++){
				viewer.setAngle(`panda_joint${ i }`, window.pos1[i-1]);
			}
		}
	});
	

	upSelect.addEventListener('change', () => viewer.up = upSelect.value);

	controlsToggle.addEventListener('click', () => controlsel.classList.toggle('hidden'));

	// watch for urdf changes
	viewer.addEventListener('urdf-change', () => {

	    Object
	        .values(sliders)
	        .forEach(sl => sl.remove());
	    sliders = {};

	});

	viewer.addEventListener('ignore-limits-change', () => {

	    Object
	        .values(sliders)
	        .forEach(sl => sl.update());

	});

	viewer.addEventListener('angle-change', e => {

	    if (sliders[e.detail]) sliders[e.detail].update();

	});

	viewer.addEventListener('joint-mouseover', e => {

	    const j = document.querySelector(`li[joint-name="${ e.detail }"]`);
	    if (j) j.setAttribute('robot-hovered', true);

	});

	viewer.addEventListener('joint-mouseout', e => {

	    const j = document.querySelector(`li[joint-name="${ e.detail }"]`);
	    if (j) j.removeAttribute('robot-hovered');

	});

	let originalNoAutoRecenter;
	viewer.addEventListener('manipulate-start', e => {

	    const j = document.querySelector(`li[joint-name="${ e.detail }"]`);
	    if (j) {
	        j.scrollIntoView({ block: 'nearest' });
	        window.scrollTo(0, 0);
	    }

	    originalNoAutoRecenter = viewer.noAutoRecenter;
	    viewer.noAutoRecenter = true;

	});

	viewer.addEventListener('manipulate-end', e => {

	    viewer.noAutoRecenter = originalNoAutoRecenter;

	});

	// create the sliders
	viewer.addEventListener('urdf-processed', () => {
		viewer.noAutoRecenter=true;

		const r = viewer.robot;
		
		const geometry = new SphereGeometry( 0.2, 32, 32 );
		const material = new MeshStandardMaterial( {color: 0x1f1fb9} );
		const sphere1 = new Mesh( geometry, material );
		const sphere2 = new Mesh( geometry, material );
		viewer.scene.add( sphere1 );
		viewer.scene.add( sphere2 );
		sphere1.position.set(1,1,1);
		sphere2.position.set(-1,1,1);
		window.obs=[0,0.02,0.63,0.57,-0.42,0.77];
		const li1 = document.createElement('li');
	            li1.innerHTML =
	            `
            <span title="x" >Obstacle1 X</span>
            <input type="range" value="0" step="0.0001" id="s1"/>
            <input type="number" step="0.0001" />
			`;
		const slider1 = li1.querySelector('input[type="range"]');
		const input1 = li1.querySelector('input[type="number"]');
		slider1.min = -1;
		slider1.max = 1;
		slider1.value=window.obs[0];
		input1.value=window.obs[0];
		sphere1.position.setX(slider1.value);
		li1.update=() => {
			input1.value=slider1.value;
			window.obs[0]=parseFloat(slider1.value);
		}
		slider1.addEventListener('input', () => {
			sphere1.position.setX(slider1.value);
			li1.update();
			viewer.redraw();
		});
		obs1.appendChild(li1);

		const li2 = document.createElement('li');
	            li2.innerHTML =
	            `
            <span title="x">Obstacle1 Y</span>
            <input type="range" value="0" step="0.0001"/>
            <input type="number" step="0.0001" />
			`;
		const slider2 = li2.querySelector('input[type="range"]');
		const input2 = li2.querySelector('input[type="number"]');
		slider2.min = -1;
		slider2.max = 1;
		slider2.value=window.obs[1];
		input2.value=window.obs[1];
		sphere1.position.setZ(-slider2.value);
		li2.update=() => {
			input2.value=slider2.value;
			window.obs[1]=parseFloat(slider2.value);
		}
		slider2.addEventListener('input', () => {
			sphere1.position.setZ(-slider2.value);
			li2.update();
			viewer.redraw();
		});
		obs1.appendChild(li2);

		const li3 = document.createElement('li');
	            li3.innerHTML =
	            `
            <span title="x">Obstacle1 Z</span>
            <input type="range" value="0" step="0.0001"/>
            <input type="number" step="0.0001" />
			`;
		const slider3 = li3.querySelector('input[type="range"]');
		const input3 = li3.querySelector('input[type="number"]');
		slider3.min = -1;
		slider3.max = 1;
		slider3.value=window.obs[2];
		input3.value=window.obs[2];
		sphere1.position.setY(slider3.value);
		li3.update=() => {
			input3.value=slider3.value;
			window.obs[2]=parseFloat(slider3.value);
		}
		slider3.addEventListener('input', () => {
			sphere1.position.setY(slider3.value);
			li3.update();
			viewer.redraw();
		});
		obs1.appendChild(li3);

		const li4 = document.createElement('li');
	            li4.innerHTML =
	            `
            <span title="x">Obstacle2 X</span>
            <input type="range" value="0" step="0.0001"/>
            <input type="number" step="0.0001" />
			`;
		const slider4 = li4.querySelector('input[type="range"]');
		const input4 = li4.querySelector('input[type="number"]');
		slider4.min = -1;
		slider4.max = 1;
		slider4.value=window.obs[3];
		input4.value=window.obs[3];
		sphere2.position.setX(slider4.value);
		li4.update=() => {
			input4.value=slider4.value;
			window.obs[3]=parseFloat(slider4.value);
		}
		slider4.addEventListener('input', () => {
			sphere2.position.setX(slider4.value);
			li4.update();
			viewer.redraw();
		});
		obs2.appendChild(li4);

		const li5 = document.createElement('li');
	            li5.innerHTML =
	            `
            <span title="x">Obstacle2 Y</span>
            <input type="range" value="0" step="0.0001"/>
            <input type="number" step="0.0001" />
			`;
		const slider5 = li5.querySelector('input[type="range"]');
		const input5 = li5.querySelector('input[type="number"]');
		slider5.min = -3;
		slider5.max = 3;
		slider5.value=window.obs[4];
		input5.value=window.obs[4];
		sphere2.position.setZ(-slider5.value);
		li5.update=() => {
			input5.value=slider5.value;
			window.obs[4]=parseFloat(slider5.value);
		}
		slider5.addEventListener('input', () => {
			sphere2.position.setZ(-slider5.value);
			li5.update();
			viewer.redraw();
		});
		obs2.appendChild(li5);

		const li6 = document.createElement('li');
	            li6.innerHTML =
	            `
            <span title="x">Obstacle2 Z</span>
            <input type="range" value="0" step="0.0001"/>
            <input type="number" step="0.0001" />
			`;
		const slider6 = li6.querySelector('input[type="range"]');
		const input6 = li6.querySelector('input[type="number"]');
		slider6.min = -1;
		slider6.max = 1;
		slider6.value=window.obs[5];
		input6.value=window.obs[5];
		sphere2.position.setY(slider6.value);
		li6.update=() => {
			input6.value=slider6.value;
			window.obs[5]=parseFloat(slider6.value);
		}
		slider6.addEventListener('input', () => {
			sphere2.position.setY(slider6.value);
			li6.update();
			viewer.redraw();
		});
		obs2.appendChild(li6);
		
	    Object
	        .keys(r.joints)
	        .sort((a, b) => {

	            const da = a.split(/[^\d]+/g).filter(v => !!v).pop();
	            const db = b.split(/[^\d]+/g).filter(v => !!v).pop();

	            if (da !== undefined && db !== undefined) {
	                const delta = parseFloat(da) - parseFloat(db);
	                if (delta !== 0) return delta;
	            }

	            if (a > b) return 1;
	            if (b > a) return -1;
	            return 0;

	        })
	        .map(key => r.joints[key])
	        .forEach(joint => {

	            const li = document.createElement('li');
	            li.innerHTML =
	            `
            <span title="${ joint.name }">${ joint.name }</span>
            <input type="range" value="0" step="0.0001"/>
            <input type="number" step="0.0001" />
            `;
	            li.setAttribute('joint-type', joint.jointType);
	            li.setAttribute('joint-name', joint.name);

	            sliderList.appendChild(li);

	            // update the joint display
	            const slider = li.querySelector('input[type="range"]');
				const input = li.querySelector('input[type="number"]');
				var ch = joint.name.charAt(joint.name.length-1);
				var index=parseInt(ch);
				slider.min = -6.28;
	            slider.max = 6.28;
				slider.value=window.pos0[index-1];
				viewer.setAngle(joint.name, window.pos0[index-1]);
	            li.update = () => {
					let degVal = joint.angle;
					var ch = joint.name.charAt(joint.name.length-1);
					var index=parseInt(ch);
					if(index<8){
						if(window.current==0){
							window.pos0[index-1]=degVal;
						}else if(window.current==1){
							window.pos1[index-1]=degVal;
						}
					}

	                if (joint.jointType === 'revolute' || joint.jointType === 'continuous') {
	                    degVal *= RAD2DEG;
	                }

	                if (Math.abs(degVal) > 1) {
	                    degVal = degVal.toFixed(1);
	                } else {
	                    degVal = degVal.toPrecision(2);
	                }

	                input.value = parseFloat(degVal);

	                // directly input the value
	                slider.value = joint.angle;

	                if (viewer.ignoreLimits || joint.jointType === 'continuous') {
	                    slider.min = -6.28;
	                    slider.max = 6.28;

	                    input.min = -6.28 * RAD2DEG;
	                    input.max = 6.28 * RAD2DEG;
	                } else {
	                    slider.min = joint.limit.lower;
	                    slider.max = joint.limit.upper;

	                    input.min = joint.limit.lower * RAD2DEG;
	                    input.max = joint.limit.upper * RAD2DEG;
	                }
	            };

	            switch (joint.jointType) {

	                case 'continuous':
	                case 'prismatic':
	                case 'revolute':
	                    break;
	                default:
	                    li.update = () => {};
	                    input.remove();
	                    slider.remove();

	            }

	            slider.addEventListener('input', () => {
	                viewer.setAngle(joint.name, slider.value);
	                li.update();
	            });

	            input.addEventListener('change', () => {
	                viewer.setAngle(joint.name, input.value * DEG2RAD);
	                li.update();
	            });

	            li.update();

	            sliders[joint.name] = li;

	        });

	});

	document.addEventListener('WebComponentsReady', () => {

	    viewer.loadMeshFunc = (path, manager, done) => {

	        const ext = path.split(/\./g).pop().toLowerCase();
	        switch (ext) {

	            case 'gltf':
	            case 'glb':
	                new GLTFLoader(manager).load(
	                    path,
	                    result => done(result.scene),
	                    null,
	                    err => done(null, err)
	                );
	                break;
	            case 'obj':
	                new OBJLoader(manager).load(
	                    path,
	                    result => done(result),
	                    null,
	                    err => done(null, err)
	                );
	                break;
	            case 'dae':
	                new ColladaLoader(manager).load(
	                    path,
	                    result => done(result.scene),
	                    null,
	                    err => done(null, err)
	                );
	                break;
	            case 'stl':
	                new STLLoader(manager).load(
	                    path,
	                    result => {
	                        const material = new MeshPhongMaterial();
	                        const mesh = new Mesh(result, material);
	                        done(mesh);
	                    },
	                    null,
	                    err => done(null, err)
	                );
	                break;

	        }

	    };

	    document.querySelector('li[urdf]').dispatchEvent(new Event('click'));

	    if (/javascript\/example\/build/i.test(window.location)) {
	        viewer.package = '../../../urdf';
	    }

	    registerDragEvents(viewer, () => {
	        setColor('#263238');
	    });

	});

	// init 2D UI and animation
	const updateAngles = () => {

	    if (!viewer.setAngle) return;

	    // reset everything to 0 first
	    

	    // animate the legs
	    const time = Date.now() / 3e2;
	    for (let i = 1; i <= 6; i++) {

	        const offset = i * Math.PI / 3;
	        const ratio = Math.max(0, Math.sin(time + offset));

	        viewer.setAngle(`HP${ i }`, MathUtils.lerp(30, 0, ratio) * DEG2RAD);
	        viewer.setAngle(`KP${ i }`, MathUtils.lerp(90, 150, ratio) * DEG2RAD);
	        viewer.setAngle(`AP${ i }`, MathUtils.lerp(-30, -60, ratio) * DEG2RAD);

	        viewer.setAngle(`TC${ i }A`, MathUtils.lerp(0, 0.065, ratio));
	        viewer.setAngle(`TC${ i }B`, MathUtils.lerp(0, 0.065, ratio));

	        viewer.setAngle(`W${ i }`, window.performance.now() * 0.001);

	    }

	};

	const updateLoop = () => {


	    requestAnimationFrame(updateLoop);

	};

	document.querySelectorAll('#urdf-options li[urdf]').forEach(el => {

	    el.addEventListener('click', e => {

	        const urdf = e.target.getAttribute('urdf');
	        const color = e.target.getAttribute('color');

	        viewer.up = '+Z';
	        document.getElementById('up-select').value = viewer.up;
	        viewer.urdf = urdf;
	        setColor(color);

	    });

	});

	document.addEventListener('WebComponentsReady', () => {


	    // stop the animation if user tried to manipulate the model
	    viewer.addEventListener('urdf-processed', e => updateAngles());
	    updateLoop();
	    viewer.camera.position.set(1.5, 1, 1.5);

	});