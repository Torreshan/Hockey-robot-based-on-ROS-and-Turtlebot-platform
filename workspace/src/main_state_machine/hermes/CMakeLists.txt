set(MOC_H
  hermes.h
  referee.h
)

set(SRC_H
  hermescodes.h
  hermes.h
  referee.h
)

set(SRC_CPP
  hermes.cpp
  referee.cpp
)

QT4_WRAP_CPP( MOC_CPP ${MOC_H} )


add_library( referee STATIC
	${SRC_CPP}
	${SRC_H}
	${MOC_CPP}
)

target_link_libraries ( referee
	${QT_LIBRARIES}
)
