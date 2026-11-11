// libfpv - browser runtime, an Emscripten JS library (--js-library, added by
// CMake for targets linking libfpv::fpv; docs/WEB.md).
//
//  - Persistent data folder: Module.libfpvDataDir (default "/libfpv") is
//    kept in the browser's IndexedDB. It is mounted and loaded before main()
//    runs, and every file libfpv closes or renames there is saved at once
//    (IDBFS autoPersist), so settings, Betaflight EEPROMs and radio mappings
//    survive a reload with no save call from the game.
//  - Clipboard for the overlay: copy writes the system clipboard; Ctrl+V and
//    the Import tab's Paste button read it (the browser may ask once).
// SPDX-License-Identifier: GPL-3.0-or-later

addToLibrary({
  $libfpvDataDir: () => Module['libfpvDataDir'] || '/libfpv',

  $libfpvMountData__deps: ['$FS', '$IDBFS', '$addRunDependency', '$removeRunDependency', '$libfpvDataDir'],
  $libfpvMountData: () => {
    // Node (the test suite) has no IndexedDB and uses real files instead.
    if (typeof indexedDB == 'undefined') return;
    var dir = libfpvDataDir();
    FS.mkdirTree(dir);
    FS.mount(IDBFS, {autoPersist: true}, dir);
    addRunDependency('libfpv-data');
    FS.syncfs(true, (e) => {
      if (e) err(`libfpv: saved settings could not be loaded (${e})`);
      removeRunDependency('libfpv-data');
    });
  },
  $libfpvMountData__postset: 'addOnPreRun(libfpvMountData);',

  libfpv_web_data_dir__deps: ['$libfpvDataDir', '$libfpvMountData', '$addOnPreRun', '$stringToNewUTF8'],
  libfpv_web_data_dir: () => stringToNewUTF8(typeof indexedDB == 'undefined' ? '' : libfpvDataDir()),

  libfpv_web_clipboard_write__deps: ['$UTF8ToString'],
  libfpv_web_clipboard_write: (text) => {
    navigator.clipboard?.writeText(UTF8ToString(text)).catch(() => {});
  },

  libfpv_web_open_url__deps: ['$UTF8ToString'],
  libfpv_web_open_url: (url) => {
    if (typeof window != 'undefined') window.open(UTF8ToString(url), '_blank', 'noopener');
  },

  libfpv_web_clipboard_read__deps: ['$stringToNewUTF8', 'libfpv_web_on_paste', 'free'],
  libfpv_web_clipboard_read: (toImport) => {
    if (!navigator.clipboard?.readText) {
      err('libfpv: this browser does not let pages read the clipboard');
      return;
    }
    navigator.clipboard.readText().then((text) => {
      var p = stringToNewUTF8(text);
      _libfpv_web_on_paste(p, toImport);
      _free(p);
    }, (e) => err(`libfpv: clipboard not readable (${e})`));
  },
});
