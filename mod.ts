import plugins, { Options } from "./plugins.ts";

import type { Site } from "lume/core.ts";

export type { Options } from "./plugins.ts";

export default function (options: Partial<Options> = {}) {
  return (site: Site) => {
    // Configure the site
    site.use(plugins(options));

    // Add remote files
    const files = [
      "_includes/css/fonts.css",
      "_includes/css/navbar.css",
      "_includes/css/page.css",
      "_includes/css/post-list.css",
      "_includes/css/post.css",
      "_includes/css/reset.css",
      "_includes/css/badge.css",
      "_includes/css/variables.css",
      "_includes/css/search.css",
      "_includes/layouts/archive_result.vto",
      "_includes/layouts/archive.vto",
      "_includes/layouts/base.vto",
      "_includes/layouts/page.vto",
      "_includes/layouts/post.vto",
      "_includes/templates/post-details.vto",
      "_includes/templates/post-list.vto",
      "fonts/inter.woff2",
      "fonts/inter-italic.woff2",
      "fonts/epilogue-bold.woff2",
      "posts/_data.yml",
      "_data.yml",
      "_data/i18n.yml",
      "404.md",
      "archive_result.tmpl.js",
      "archive.tmpl.js",
      "index.vto",
      "styles.css",
      "favicon.png",
    ];

    for (const file of files) {
      site.remoteFile(file, import.meta.resolve(`./src/${file}`));
    }
  };
}
