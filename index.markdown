---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: homepage
---

Welcome to my personal website. Feel free to browse around some of my project work under the [projects](/projects/) section or if you want to take a look at the latest rabbit hole I've gone into, head on over to the [essays](/essays/) section.

Hopefully you find something interesting!

## Projects

{% assign latest_projects = site.projects | sort: 'date' | reverse | slice: 0, 3 %}
<ul class="home-projects">
{% for project in latest_projects %}
	<li>
		<a href="{{ project.url }}">{{ project.title | escape }}</a>
		{%- if project.excerpt -%}
			<div>
				{{ project.excerpt | strip_html | strip | truncate: 220 }}
			</div>
		{%- endif -%}
	</li>
{% endfor %}
</ul>

[See all projects →](/projects/)

## Essays

{% assign latest_posts = site.posts | slice: 0, 3 %}
<ul>
{% for post in latest_posts %}
	<li><a href="{{ post.url }}">{{ post.title | escape }}</a> ({{ post.date | date: "%Y-%m-%d" }})</li>
{% endfor %}
</ul>

[See all essays →](/essays/)