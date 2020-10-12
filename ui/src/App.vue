<template>
  <v-app>
    <v-app>
      <v-app-bar
        color="indigo"
        app
        dark
        clipped-left
      >
        <v-app-bar-nav-icon @click.stop="drawer = !drawer"></v-app-bar-nav-icon>
        <v-toolbar-title>BeagleGUI</v-toolbar-title>
      </v-app-bar>

      <v-navigation-drawer
        v-model="drawer"
        app
        clipped
      >
        <v-list dense>
          <v-list-item link>
            <v-list-item-action>
              <v-icon>mdi-rotate-3d</v-icon>
            </v-list-item-action>
            <v-list-item-content>
              <v-list-item-title>3D View</v-list-item-title>
            </v-list-item-content>
          </v-list-item>
        </v-list>
      </v-navigation-drawer>

      <v-main>
        <router-view></router-view>
      </v-main>
    </v-app>
  </v-app>
</template>

<script>

export default {
  name: 'App',

  components: {
  },

  data: () => ({
    drawer: false
  })
}
</script>

<style>
body {
  margin: 0;
  padding: 0;
}
</style>
