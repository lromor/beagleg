import { Observable } from 'rxjs'

var decoder = new TextDecoder('utf-8')

// Returns an rxjs observable
// which ends when the stream ends.
function getStream () {
  const obs = Observable.create(observer => {
    fetch('/api/stream').then(function (response) {
      const reader = response.body.getReader()
      var buffer = ''
      function go () {
        reader.read().then(function (result) {
          if (result.done) {
            observer.complete()
            return
          }
          buffer += decoder.decode(result.value)
          var res = buffer.split('\n')
          while (res.length > 1) {
            observer.next(JSON.parse(res.shift()))
          }
          buffer = res[0]
          go()
        })
      }
      go()
    })
  })
  return obs
}

export default getStream
