README-ja.md: drone-okazaki-isogai-theorem.md
	sed '/^<!/! s/\\bm{/\\boldsymbol{/g' $^ > README-ja.md

reverse:
	sed '/^<!/! s/\\boldsymbol{/\\bm{/g' README-ja.md > reversed-drone-okazaki-isogai-theorem.md
