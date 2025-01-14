import date, { Options as DateOptions } from "lume/plugins/date.ts";
import postcss from "lume/plugins/postcss.ts";
import terser from "lume/plugins/terser.ts";
import prism, { Options as PrismOptions } from "lume/plugins/prism.ts";
import basePath from "lume/plugins/base_path.ts";
import slugifyUrls from "lume/plugins/slugify_urls.ts";
import resolveUrls from "lume/plugins/resolve_urls.ts";
import metas from "lume/plugins/metas.ts";
import pagefind, { Options as PagefindOptions } from "lume/plugins/pagefind.ts";
import sitemap from "lume/plugins/sitemap.ts";
import feed from "lume/plugins/feed.ts";
import vento from "lume/plugins/vento.ts";
import toc from "https://deno.land/x/lume_markdown_plugins@v0.5.0/toc.ts";
import image from "https://deno.land/x/lume_markdown_plugins@v0.5.0/image.ts";
import footnotes from "https://deno.land/x/lume_markdown_plugins@v0.5.0/footnotes.ts";
import katex from "lume/plugins/katex.ts";

import type { Page, Site } from "lume/core.ts";

export interface Options {
  prism?: Partial<PrismOptions>;
  date?: Partial<DateOptions>;
  pagefind?: Partial<PagefindOptions>;
}

/** Configure the site */
export default function (options: Options = {}) {
  return (site: Site) => {
    site.use(postcss())
      .use(basePath())
      .use(toc())
      .use(footnotes())
      .use(prism(options.prism))
      .use(date(options.date))
      .use(metas())
      .use(image())
      .use(resolveUrls())
      .use(slugifyUrls())
      .use(pagefind(options.pagefind))
      .use(terser())
      .use(sitemap())
      .use(vento())
      .use(katex())
      .use(feed({
        output: ["/feed.xml", "/feed.json"],
        query: "type=post",
        info: {
          title: "=metas.site",
          description: "=metas.description",
        },
        items: {
          title: "=title",
        },
      }))
      .copy("fonts")
      .copy("favicon.png")
      .copy("image")
      .preprocess([".md"], (page: Page) => {
        page.data.excerpt ??= (page.data.content as string).split(
          /<!--\s*more\s*-->/i,
        )[0];
      });

    // Basic CSS Design System
    site.remoteFile(
      "_includes/css/ds.css",
      "https://unpkg.com/@lumeland/ds@0.2.4/ds.css",
    );
  };
}
