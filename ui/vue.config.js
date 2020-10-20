module.exports = {
  productionSourceMap: process.env.NODE_ENV !== 'production',
  transpileDependencies: [
    'vuetify'
  ],
  devServer: {
    proxy: {
      '^/api': {
        target: 'http://localhost:6698',
        // changeOrigin: true,
        headers: {
          Connection: 'keep-alive'
        },
        secure: false
      }
    }
  }
}
