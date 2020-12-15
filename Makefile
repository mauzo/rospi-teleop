sinclude config.mk

bin_dir=        /usr/bin
bin_files=      rospi-teleop

clean_files+=     ${bin_files}
distclean_files+= config.mk

install_prog=   install -o root -g root -m 755

subst+=         -e's!@ROS_DIR@!${ROS_DIR}!g'
subst+=         -e's!@PYTHON_CMD@!${PYTHON_CMD}!g'

%: %.in
	sed ${subst} <$< >$@

all: ${bin_files}
	chmod a+x ${bin_files}

install:
	for f in ${bin_files}; do \
	    ${install_bin} "$f" "$bin_dir"; \
	done

clean:
	rm -f ${clean_files}

distclean: clean
	rm -f ${distclean_files}
