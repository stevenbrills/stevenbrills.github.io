---
layout: page
title: Projects
permalink: /projects/
body_class: projects-listing
---

<div class="projects-listing">
{% for project in site.projects %}
  <h2><a href="{{ project.url }}">{{ project.title }}</a></h2>
  {{ project.excerpt | markdownify }}
{% endfor %}
</div>
