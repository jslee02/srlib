MAKE=make
MD=mkdir
LIB_COMMON=common
LIB_LIEGROUP=LieGroup
LIB_SRDYN=srDyn
LIB_SRG=srg
LIB_RENDERER=Renderer


default:
	${MAKE} -C ${LIB_COMMON}
	${MAKE} -C ${LIB_LIEGROUP}
	${MAKE} -C ${LIB_SRDYN}
	${MAKE} -C ${LIB_SRG}
	${MAKE} -C ${LIB_RENDERER}

new:
	${MAKE} -C ${LIB_COMMON} new
	${MAKE} -C ${LIB_LIEGROUP} new
	${MAKE} -C ${LIB_SRDYN} new
	${MAKE} -C ${LIB_SRG} new
	${MAKE} -C ${LIB_RENDERER} new

clean:
	${MAKE} -C ${LIB_COMMON} clean
	${MAKE} -C ${LIB_LIEGROUP} clean
	${MAKE} -C ${LIB_SRDYN} clean
	${MAKE} -C ${LIB_SRG} clean
	${MAKE} -C ${LIB_RENDERER} clean
	
deploy:
	${MD} -p ../include
	${MD} -p ../lib
	${MAKE} -C ${LIB_COMMON} deploy
	${MAKE} -C ${LIB_LIEGROUP} deploy
	${MAKE} -C ${LIB_SRDYN} deploy
	${MAKE} -C ${LIB_SRG} deploy
	${MAKE} -C ${LIB_RENDERER} deploy

${LIB_COMMON}:
	${MAKE} -C ${LIB_COMMON} new

${LIB_LIEGROUP}:
	${MAKE} -C ${LIB_LIEGROUP} new
	
${LIB_SRDYN}:
	${MAKE} -C ${LIB_SRDYN} new
	
${LIB_SRG}:
	${MAKE} -C ${LIB_SRG} new
	
${LIB_RENDERER}:
	${MAKE} -C ${LIB_RENDERER} new
	
