<template>
  <div id="container">
  </div>
</template>

<script>
import * as Three from 'three'
import { TrackballControls } from 'three/examples/jsm/controls/TrackballControls.js'

export default {
  name: 'ThreeDimView',
  data () {
    return {
      camera: null,
      scene: null,
      renderer: null,
      mesh: null,
      controls: null
    }
  },
  methods: {
    init: function () {
      const container = document.getElementById('container')

      this.camera = new Three.PerspectiveCamera(70, container.clientWidth / container.clientHeight, 0.01, 100)
      this.camera.position.z = 1
      this.camera.position.x = 1
      this.camera.position.y = 0.5
      this.camera.lookAt(0.0, 1, 0.0)

      this.scene = new Three.Scene()

      this.scene.background = new Three.Color(0xf0f0f0)

      var helper = new Three.GridHelper(5, 100)
      helper.position.y = 0
      helper.material.opacity = 0.25
      helper.material.transparent = true
      this.scene.add(helper)

      const geometry = new Three.BoxGeometry(0.2, 0.2, 0.2)
      const material = new Three.MeshNormalMaterial()

      this.mesh = new Three.Mesh(geometry, material)
      this.scene.add(this.mesh)

      this.renderer = new Three.WebGLRenderer({ antialias: true })
      this.renderer.setSize(container.clientWidth, container.clientHeight, false)

      container.appendChild(this.renderer.domElement)
      this.controls = new TrackballControls(this.camera, this.renderer.domElement)

      this.controls.rotateSpeed = 1.0
      this.controls.zoomSpeed = 1.2
      this.controls.panSpeed = 0.8

      this.controls.keys = [65, 83, 68]
    },

    resizeCanvasToDivSize: function () {
      const container = document.getElementById('container')
      const canvas = this.renderer.domElement

      // look up the size the canvas is being displayed
      const width = container.clientWidth
      const height = container.clientHeight

      // adjust displayBuffer size to match
      if (canvas.width !== width || canvas.height !== height) {
        // you must pass false here or three.js sadly fights the browser
        this.renderer.setSize(width, height, false)
        this.camera.aspect = width / height
        this.camera.updateProjectionMatrix()
        this.controls.handleResize()
      }
    },

    animate: function () {
      this.resizeCanvasToDivSize()

      requestAnimationFrame(this.animate)

      this.controls.update()
      // this.mesh.rotation.x += 0.01
      // this.mesh.rotation.y += 0.02
      this.renderer.render(this.scene, this.camera)
    }
  },
  mounted () {
    this.init()
    this.animate()
  }
}
</script>

<style>
  #container {
    width: 100%;
    height: 100%;
    line-height: 0;
  }

  canvas {
    position: relative;
  }
</style>
