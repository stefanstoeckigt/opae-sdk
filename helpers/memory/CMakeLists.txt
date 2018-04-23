project(libdmamem)

enable_language(C ASM)

set(ASM_OPTIONS "-x assembler-with-cpp")
set(CMAKE_ASM_FLAGS "${CFLAGS} ${ASM_OPTIONS}")

set(DMA_MEM_SRC
   ase_memory.c
   fpga_dma.c
   memcpy_s_fast.c
   mmio_block.c
   x86-sse2.S
)

add_library(intel_opae_dma_helper STATIC ${DMA_MEM_SRC})

target_link_libraries(intel_opae_dma_helper
   safestr
   -lpthread
   -lopae-c
)

install(TARGETS intel_opae_dma_helper
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   COMPONENT libdmamem 
)
