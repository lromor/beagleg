<template>
  <div id="container">
  </div>
</template>

<script>
import * as Three from 'three'

export default {
  name: 'ThreeDimView',
  data () {
    return {
      camera: null,
      scene: null,
      renderer: null,
      mesh: null
    }
  },
  methods: {
    init: function () {
      const container = document.getElementById('container')

      this.camera = new Three.PerspectiveCamera(70, container.clientWidth / container.clientHeight, 0.01, 10)
      this.camera.position.z = 1

      this.scene = new Three.Scene()

      const geometry = new Three.BoxGeometry(0.2, 0.2, 0.2)
      const material = new Three.MeshNormalMaterial()

      this.mesh = new Three.Mesh(geometry, material)
      this.scene.add(this.mesh)

      this.renderer = new Three.WebGLRenderer({ antialias: true })
      this.renderer.setSize(container.clientWidth, container.clientHeight, false)
      container.appendChild(this.renderer.domElement)
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
      }
    },

    animate: function () {
      this.resizeCanvasToDivSize()

      requestAnimationFrame(this.animate)
      this.mesh.rotation.x += 0.01
      this.mesh.rotation.y += 0.02
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
    background: blue;
    line-height: 0;
  }

  canvas {
    position: relative;
  }
</style>
