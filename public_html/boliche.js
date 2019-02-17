	function iniciarRampa() {
		var loader = new THREE.GLTFLoader();
		loader.load('boliche.glb', function (gltf) {
			console.log(gltf);

			gltf.scene.getObjectByName('luz1').castShadow = true;
			scene.add(gltf.scene.getObjectByName('luz1'));

			initPhysics();
			var geometry;
			var vertices;
			var groundShape;
			var groundTransform;
			var groundMass;
			var groundLocalInertia;
			var groundMotionState;
			var groundBody;

			// ******************************** ESQUERDA
			heightData = generateHeight(terrainWidth, terrainDepth, terrainMinHeight, terrainMaxHeight);
			geometry = new THREE.PlaneBufferGeometry(terrainWidthExtents, terrainDepthExtents, terrainWidth - 1, terrainDepth - 1);
			geometry.rotateX(-Math.PI / 2);
			geometry.translate(-2.2, 0, 0);
			vertices = geometry.attributes.position.array;
			for (var i = 0, j = 0, l = vertices.length; i < l; i++, j += 3) {
				// j + 1 because it is the y component that we modify
				vertices[j + 1] = heightData[i];
			}
			geometry.computeVertexNormals();
			terrainMesh = new THREE.Mesh(geometry, gltf.scene.getObjectByName('esquerda').material);
			terrainMesh.receiveShadow = true;
			scene.add(terrainMesh);
			// Create the terrain body
			groundShape = createTerrainShape();
			groundTransform = new Ammo.btTransform();
			groundTransform.setIdentity();
			// Shifts the terrain, since bullet re-centers it on its bounding box.
			groundTransform.setOrigin(new Ammo.btVector3(-2.2, (terrainMaxHeight + terrainMinHeight) / 2, 0));
			groundMass = 0;
			groundLocalInertia = new Ammo.btVector3(0, 0, 0);
			groundMotionState = new Ammo.btDefaultMotionState(groundTransform);
			groundBody = new Ammo.btRigidBody(new Ammo.btRigidBodyConstructionInfo(groundMass, groundMotionState, groundShape, groundLocalInertia));
			physicsWorld.addRigidBody(groundBody);


			// ******************************** MEIO
			terrainWidthExtents = 2.8;
			terrainDepthExtents = 30;
			terrainWidth = 2;
			terrainDepth = 2;
			terrainHalfWidth = terrainWidth / 2;
			terrainHalfDepth = terrainDepth / 2;

			heightData = generateHeight(terrainWidth, terrainDepth, terrainMinHeight, terrainMaxHeight);
			geometry = new THREE.PlaneBufferGeometry(terrainWidthExtents, terrainDepthExtents, terrainWidth - 1, terrainDepth - 1);
			geometry.rotateX(-Math.PI / 2);
			geometry.translate(0, 0, -2.5);
			vertices = geometry.attributes.position.array;
			for (var i = 0, j = 0, l = vertices.length; i < l; i++, j += 3) {
				// j + 1 because it is the y component that we modify
				vertices[j + 1] = heightData[i];
			}
			geometry.computeVertexNormals();
			terrainMesh = new THREE.Mesh(geometry, gltf.scene.getObjectByName('meio').material);
			terrainMesh.receiveShadow = true;
			scene.add(terrainMesh);
			// Create the terrain body
			groundShape = createTerrainShape();
			groundTransform = new Ammo.btTransform();
			groundTransform.setIdentity();
			// Shifts the terrain, since bullet re-centers it on its bounding box.
			groundTransform.setOrigin(new Ammo.btVector3(0, (terrainMaxHeight + terrainMinHeight) / 2, -2.5));
			groundMass = 0;
			groundLocalInertia = new Ammo.btVector3(0, 0, 0);
			groundMotionState = new Ammo.btDefaultMotionState(groundTransform);
			groundBody = new Ammo.btRigidBody(new Ammo.btRigidBodyConstructionInfo(groundMass, groundMotionState, groundShape, groundLocalInertia));
			physicsWorld.addRigidBody(groundBody);


			// ******************************** DIREITA
			terrainWidthExtents = 1.6;
			terrainDepthExtents = 25;
			terrainWidth = 128;
			terrainDepth = 2;
			terrainHalfWidth = terrainWidth / 2;
			terrainHalfDepth = terrainDepth / 2;

			heightData = generateHeight(terrainWidth, terrainDepth, terrainMinHeight, terrainMaxHeight);
			geometry = new THREE.PlaneBufferGeometry(terrainWidthExtents, terrainDepthExtents, terrainWidth - 1, terrainDepth - 1);
			geometry.rotateX(-Math.PI / 2);
			geometry.translate(2.2, 0, 0);
			vertices = geometry.attributes.position.array;
			for (var i = 0, j = 0, l = vertices.length; i < l; i++, j += 3) {
				// j + 1 because it is the y component that we modify
				vertices[j + 1] = heightData[i];
			}
			geometry.computeVertexNormals();
			terrainMesh = new THREE.Mesh(geometry, gltf.scene.getObjectByName('direita').material);
			terrainMesh.receiveShadow = true;
			scene.add(terrainMesh);
			// Create the terrain body
			groundShape = createTerrainShape();
			groundTransform = new Ammo.btTransform();
			groundTransform.setIdentity();
			// Shifts the terrain, since bullet re-centers it on its bounding box.
			groundTransform.setOrigin(new Ammo.btVector3(2.2, (terrainMaxHeight + terrainMinHeight) / 2, 0));
			groundMass = 0;
			groundLocalInertia = new Ammo.btVector3(0, 0, 0);
			groundMotionState = new Ammo.btDefaultMotionState(groundTransform);
			groundBody = new Ammo.btRigidBody(new Ammo.btRigidBodyConstructionInfo(groundMass, groundMotionState, groundShape, groundLocalInertia));
			physicsWorld.addRigidBody(groundBody);

			
			// ************************ PINO
			var posx = 0;
			var x = 1;
			for (var i = 0; i < 4; i++) {
				for (var j = 0; j < x; j++) {
					geometry = gltf.scene.getObjectByName('pino').geometry;
					vertices = geometry.attributes.position.array;
					terrainMesh = new THREE.Mesh(geometry, gltf.scene.getObjectByName('pino').material);

					terrainMesh.position.set(posx + (j * 0.65), 0.620636, -14.5 - (i * 0.65));
					
					terrainMesh.castShadow = true;
					terrainMesh.receiveShadow = true;
					scene.add(terrainMesh);
					// Create the terrain body
					groundShape = createConvexHullPhysicsShape(vertices);	
					groundTransform = new Ammo.btTransform();
					groundTransform.setIdentity();
					// Shifts the terrain, since bullet re-centers it on its bounding box.
					groundTransform.setOrigin(new Ammo.btVector3(terrainMesh.position.x, terrainMesh.position.y, terrainMesh.position.z));
					groundMass = 2000000;
					groundLocalInertia = new Ammo.btVector3(0, 0, 0);
					groundShape.calculateLocalInertia( groundMass, groundLocalInertia );
					groundMotionState = new Ammo.btDefaultMotionState(groundTransform);
					groundBody = new Ammo.btRigidBody(new Ammo.btRigidBodyConstructionInfo(groundMass, groundMotionState, groundShape, groundLocalInertia));
					physicsWorld.addRigidBody(groundBody);
					terrainMesh.userData.physicsBody = groundBody;
					terrainMesh.userData.collided = false;
					if (groundMass > 0) {
						// Disable deactivation
						groundBody.setActivationState(4);
					}
					dynamicObjects.push(terrainMesh);
				}
				x++;
				posx -= 0.32;
			}
		}, undefined, function (e) {
			console.error(e);
		});
	}

	// Heightfield parameters
	var terrainWidthExtents = 1.6;
	var terrainDepthExtents = 25;
	var terrainWidth = 128;
	var terrainDepth = 2;
	var terrainHalfWidth = terrainWidth / 2;
	var terrainHalfDepth = terrainDepth / 2;
	var terrainMaxHeight;
	var terrainMinHeight;
	var tempBtVec3_1 = new Ammo.btVector3( 0, 0, 0 );

	// Graphics variables
	
	var terrainMesh;	
	

	// Physics variables
	var collisionConfiguration;
	var dispatcher;
	var broadphase;
	var solver;
	var physicsWorld;
	var dynamicObjects = [];
	var transformAux1 = new Ammo.btTransform();
	var heightData = null;
	var ammoHeightData = null;

	var mouseCoords = new THREE.Vector2();
	var raycaster = new THREE.Raycaster();
	var ballMaterial = new THREE.MeshPhongMaterial({color: 0x202020});
	
	

	function initPhysics() {
		// Physics configuration
		collisionConfiguration = new Ammo.btDefaultCollisionConfiguration();
		dispatcher = new Ammo.btCollisionDispatcher(collisionConfiguration);
		broadphase = new Ammo.btDbvtBroadphase();
		solver = new Ammo.btSequentialImpulseConstraintSolver();
		physicsWorld = new Ammo.btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
		physicsWorld.setGravity(new Ammo.btVector3(0, -20, 0));
	}
	function generateHeight(width, depth, minHeight, maxHeight) {
		// Generates the height data (a sinus wave)
		var size = width * depth;
		var data = new Float32Array(size);
		var p = 0;
		for (var j = 0; j < depth; j++) {
			for (var i = 0; i < width; i++) {
				var max = width - 1;
				var perc = i / max;
				var ang = perc * Math.PI;
				data[p] = 0.7 * Math.sin(ang) * -1;
				p++;
			}
		}
		var maior = data[0];
		var menor = data[0];
		for (var i = 0; i < data.length; i++) {
			if (data[i] > maior) {
				maior = data[i];
			}
			if (data[i] < menor) {
				menor = data[i];
			}
		}
		terrainMaxHeight = maior;
		terrainMinHeight = menor;
		return data;
	}
	function createTerrainShape() {
		// This parameter is not really used, since we are using PHY_FLOAT height data type and hence it is ignored
		var heightScale = 1;
		// Up axis = 0 for X, 1 for Y, 2 for Z. Normally 1 = Y is used.
		var upAxis = 1;
		// hdt, height data type. "PHY_FLOAT" is used. Possible values are "PHY_FLOAT", "PHY_UCHAR", "PHY_SHORT"
		var hdt = "PHY_FLOAT";
		// Set this to your needs (inverts the triangles)
		var flipQuadEdges = false;
		// Creates height data buffer in Ammo heap
		ammoHeightData = Ammo._malloc(4 * terrainWidth * terrainDepth);
		// Copy the javascript height data array to the Ammo one.
		var p = 0;
		var p2 = 0;
		for (var j = 0; j < terrainDepth; j++) {
			for (var i = 0; i < terrainWidth; i++) {
				// write 32-bit float data to memory
				Ammo.HEAPF32[ammoHeightData + p2 >> 2] = heightData[p];
				p++;
				// 4 bytes/float
				p2 += 4;
			}
		}
		// Creates the heightfield physics shape
		var heightFieldShape = new Ammo.btHeightfieldTerrainShape(
			terrainWidth,
			terrainDepth,
			ammoHeightData,
			heightScale,
			terrainMinHeight,
			terrainMaxHeight,
			upAxis,
			hdt,
			flipQuadEdges
		);
		// Set horizontal scale
		var scaleX = terrainWidthExtents / (terrainWidth - 1);
		var scaleZ = terrainDepthExtents / (terrainDepth - 1);
		heightFieldShape.setLocalScaling(new Ammo.btVector3(scaleX, 1, scaleZ));
		heightFieldShape.setMargin(0.05);
		return heightFieldShape;
	}
	
	function updatePhysics(deltaTime) {
		physicsWorld.stepSimulation(deltaTime, 10);
		// Update objects
		for (var i = 0, il = dynamicObjects.length; i < il; i++) {
			var objThree = dynamicObjects[i];
			var objPhys = objThree.userData.physicsBody;
			var ms = objPhys.getMotionState();
			if (ms) {
				ms.getWorldTransform(transformAux1);
				var p = transformAux1.getOrigin();
				var q = transformAux1.getRotation();
				objThree.position.set(p.x(), p.y(), p.z());
				objThree.quaternion.set(q.x(), q.y(), q.z(), q.w());
			}
		}
	}
	window.addEventListener('mousedown', function(event) {
		mouseCoords.set(
			(event.clientX / window.innerWidth) * 2 - 1,
			-(event.clientY / window.innerHeight) * 2 + 1
		);
		raycaster.setFromCamera(mouseCoords, camera);
		var pos = new THREE.Vector3();
		var quat = new THREE.Quaternion();
		// Creates a ball and throws it
		var ballMass = 50000000;
		var ballRadius = 0.3;
		var ball = new THREE.Mesh(new THREE.SphereBufferGeometry(ballRadius, 14, 10), ballMaterial);
		ball.castShadow = true;
		ball.receiveShadow = true;
		var ballShape = new Ammo.btSphereShape(ballRadius);
		ballShape.setMargin(0.05);
		pos.copy(raycaster.ray.direction);
		pos.add(raycaster.ray.origin);
		quat.set(0, 0, 0, 1);
		var ballBody = createRigidBody(ball, ballShape, ballMass, pos, quat);
		pos.copy(raycaster.ray.direction);
		pos.multiplyScalar(24);
		ballBody.setLinearVelocity(new Ammo.btVector3(pos.x, pos.y, pos.z));
	}, false);
	function createRigidBody(object, physicsShape, mass, pos, quat, vel, angVel) {
		if (pos) {
			object.position.copy(pos);
		} else {
			pos = object.position;
		}
		if (quat) {
			object.quaternion.copy(quat);
		} else {
			quat = object.quaternion;
		}
		var transform = new Ammo.btTransform();
		transform.setIdentity();
		transform.setOrigin(new Ammo.btVector3(pos.x, pos.y, pos.z));
		transform.setRotation(new Ammo.btQuaternion(quat.x, quat.y, quat.z, quat.w));
		var motionState = new Ammo.btDefaultMotionState(transform);
		var localInertia = new Ammo.btVector3(0, 0, 0);
		physicsShape.calculateLocalInertia(mass, localInertia);
		var rbInfo = new Ammo.btRigidBodyConstructionInfo(mass, motionState, physicsShape, localInertia);
		var body = new Ammo.btRigidBody(rbInfo);
		body.setFriction(0.5);
		if (vel) {
			body.setLinearVelocity(new Ammo.btVector3(vel.x, vel.y, vel.z));
		}
		if (angVel) {
			body.setAngularVelocity(new Ammo.btVector3(angVel.x, angVel.y, angVel.z));
		}
		object.userData.physicsBody = body;
		object.userData.collided = false;
		scene.add(object);
		if (mass > 0) {
			// Disable deactivation
			body.setActivationState(4);
		}
		dynamicObjects.push(object);
		physicsWorld.addRigidBody(body);
		return body;
	}

	function createConvexHullPhysicsShape( coords ) {

			var shape = new Ammo.btConvexHullShape();

			for ( var i = 0, il = coords.length; i < il; i+= 3 ) {
				tempBtVec3_1.setValue( coords[ i ], coords[ i + 1 ], coords[ i + 2 ] );
				var lastOne = ( i >= ( il - 3 ) );
				shape.addPoint( tempBtVec3_1, lastOne );
			}

			return shape;

		}

		function createObjectMaterial() {

				var c = Math.floor( Math.random() * ( 1 << 24 ) );
				return new THREE.MeshPhongMaterial( { color: c } );

			}