/* Generated by ./xlat/gen.sh from ./xlat/cap_version.in; do not edit. */

static const struct xlat cap_version[] = {
#if defined(_LINUX_CAPABILITY_VERSION_1) || (defined(HAVE_DECL__LINUX_CAPABILITY_VERSION_1) && HAVE_DECL__LINUX_CAPABILITY_VERSION_1)
	XLAT(_LINUX_CAPABILITY_VERSION_1),
#endif
#if defined(_LINUX_CAPABILITY_VERSION_2) || (defined(HAVE_DECL__LINUX_CAPABILITY_VERSION_2) && HAVE_DECL__LINUX_CAPABILITY_VERSION_2)
	XLAT(_LINUX_CAPABILITY_VERSION_2),
#endif
#if defined(_LINUX_CAPABILITY_VERSION_3) || (defined(HAVE_DECL__LINUX_CAPABILITY_VERSION_3) && HAVE_DECL__LINUX_CAPABILITY_VERSION_3)
	XLAT(_LINUX_CAPABILITY_VERSION_3),
#endif
	XLAT_END
};