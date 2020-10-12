<template>
  <v-container
    class="fill-height pa-0"
    fluid
  >
    <v-row class="fill-height pa-0 ma-0">
      <v-col class="pa-0">
        <three-dim-view/>
      </v-col>
    </v-row>
  </v-container>
</template>

<script>
// @ is an alias to /src
import ThreeDimView from '@/components/ThreeDimView.vue'

export default {
  name: 'Home',
  components: {
    ThreeDimView
  }
}
</script>
<style scoped>
</style>
