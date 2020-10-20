import * as Three from 'three'

class TrajectoryPiece extends Three.Line {
  static MAX_POINTS = 50

  constructor () {
    const material = new Three.LineBasicMaterial({ color: 0x0000ff })
    var geometry = new Three.BufferGeometry()

    // attributes
    var positions = new Float32Array(TrajectoryPiece.MAX_POINTS * 3)
    geometry.setAttribute('position', new Three.BufferAttribute(positions, 3))
    geometry.setDrawRange(0, 0)
    super(geometry, material)
    this.length = 0
  }

  // Return false if line piece is full.
  addPoint (point) {
    var positions = this.geometry.attributes.position.array
    const base = 3 * this.length++
    positions[base + 0] = point.x
    positions[base + 1] = point.y
    positions[base + 2] = point.z
    this.geometry.setDrawRange(0, this.length)
    this.geometry.attributes.position.needsUpdate = true
    return true
  }

  isFull () {
    return this.length === this.MAX_POINTS
  }
}

// We can't render indefinitely growing buffers efficiently.
// What we can do, is to add more lines!
// https://threejs.org/docs/#manual/en/introduction/How-to-update-things
class Trajectory {
  constructor (scene) {
    this.scene = scene
    this.pieces = []
    this.addPiece()

    // First vertex
    this.last_point = new Three.Vector3(0, 0, 0)
    this.addPoint(this.last_point)
  }

  addPoint (point) {
    let last = this.pieces[this.pieces.length - 1]
    if (last.isFull()) {
      last = this.addPiece()
      last.addPoint(this.last_point)
    }

    console.log('Adding point: ', point)
    last.addPoint(point)
    this.last_point = point
  }

  addPiece () {
    const piece = new TrajectoryPiece()
    this.scene.add(piece)
    this.pieces.push(piece)
    return piece
  }
}

function mm2space (point) {
  point.x = point.x / 1000
  point.y = point.y / 1000
  point.z = point.z / 1000
  return point
}

export { Trajectory, mm2space }
