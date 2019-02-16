pragma Warnings (Off);
pragma Ada_95;
pragma Source_File_Name (ada_main, Spec_File_Name => "b__main.ads");
pragma Source_File_Name (ada_main, Body_File_Name => "b__main.adb");
pragma Suppress (Overflow_Check);

with System.Restrictions;

package body ada_main is

   E005 : Short_Integer; pragma Import (Ada, E005, "ada__tags_E");
   E075 : Short_Integer; pragma Import (Ada, E075, "system__soft_links_E");
   E073 : Short_Integer; pragma Import (Ada, E073, "system__exception_table_E");
   E061 : Short_Integer; pragma Import (Ada, E061, "system__bb__timing_events_E");
   E118 : Short_Integer; pragma Import (Ada, E118, "ada__streams_E");
   E126 : Short_Integer; pragma Import (Ada, E126, "system__finalization_root_E");
   E124 : Short_Integer; pragma Import (Ada, E124, "ada__finalization_E");
   E128 : Short_Integer; pragma Import (Ada, E128, "system__storage_pools_E");
   E121 : Short_Integer; pragma Import (Ada, E121, "system__finalization_masters_E");
   E145 : Short_Integer; pragma Import (Ada, E145, "ada__real_time_E");
   E130 : Short_Integer; pragma Import (Ada, E130, "system__pool_global_E");
   E195 : Short_Integer; pragma Import (Ada, E195, "system__tasking__protected_objects_E");
   E201 : Short_Integer; pragma Import (Ada, E201, "system__tasking__protected_objects__multiprocessors_E");
   E214 : Short_Integer; pragma Import (Ada, E214, "system__tasking__restricted__stages_E");
   E140 : Short_Integer; pragma Import (Ada, E140, "bmp_fonts_E");
   E233 : Short_Integer; pragma Import (Ada, E233, "cortex_m__cache_E");
   E219 : Short_Integer; pragma Import (Ada, E219, "hal__audio_E");
   E242 : Short_Integer; pragma Import (Ada, E242, "hal__block_drivers_E");
   E141 : Short_Integer; pragma Import (Ada, E141, "hal__gpio_E");
   E191 : Short_Integer; pragma Import (Ada, E191, "hal__i2c_E");
   E223 : Short_Integer; pragma Import (Ada, E223, "hal__real_time_clock_E");
   E237 : Short_Integer; pragma Import (Ada, E237, "hal__sdmmc_E");
   E116 : Short_Integer; pragma Import (Ada, E116, "hal__spi_E");
   E133 : Short_Integer; pragma Import (Ada, E133, "hal__time_E");
   E256 : Short_Integer; pragma Import (Ada, E256, "cs43l22_E");
   E253 : Short_Integer; pragma Import (Ada, E253, "hal__uart_E");
   E135 : Short_Integer; pragma Import (Ada, E135, "ili9341_extended_E");
   E152 : Short_Integer; pragma Import (Ada, E152, "lis3dsh_E");
   E258 : Short_Integer; pragma Import (Ada, E258, "lis3dsh__spi_E");
   E143 : Short_Integer; pragma Import (Ada, E143, "ravenscar_time_E");
   E235 : Short_Integer; pragma Import (Ada, E235, "sdmmc_init_E");
   E161 : Short_Integer; pragma Import (Ada, E161, "stm32__adc_E");
   E167 : Short_Integer; pragma Import (Ada, E167, "stm32__dac_E");
   E207 : Short_Integer; pragma Import (Ada, E207, "stm32__dma__interrupts_E");
   E180 : Short_Integer; pragma Import (Ada, E180, "stm32__exti_E");
   E225 : Short_Integer; pragma Import (Ada, E225, "stm32__power_control_E");
   E222 : Short_Integer; pragma Import (Ada, E222, "stm32__rtc_E");
   E245 : Short_Integer; pragma Import (Ada, E245, "stm32__spi_E");
   E247 : Short_Integer; pragma Import (Ada, E247, "stm32__spi__dma_E");
   E187 : Short_Integer; pragma Import (Ada, E187, "stm32__i2c_E");
   E251 : Short_Integer; pragma Import (Ada, E251, "stm32__usarts_E");
   E241 : Short_Integer; pragma Import (Ada, E241, "stm32__sdmmc_interrupt_E");
   E218 : Short_Integer; pragma Import (Ada, E218, "stm32__i2s_E");
   E193 : Short_Integer; pragma Import (Ada, E193, "stm32__i2c__dma_E");
   E173 : Short_Integer; pragma Import (Ada, E173, "stm32__gpio_E");
   E230 : Short_Integer; pragma Import (Ada, E230, "stm32__sdmmc_E");
   E178 : Short_Integer; pragma Import (Ada, E178, "stm32__syscfg_E");
   E156 : Short_Integer; pragma Import (Ada, E156, "stm32__device_E");
   E154 : Short_Integer; pragma Import (Ada, E154, "stm32__setup_E");
   E150 : Short_Integer; pragma Import (Ada, E150, "stm32__board_E");
   E260 : Short_Integer; pragma Import (Ada, E260, "stm32__user_button_E");

   Sec_Default_Sized_Stacks : array (1 .. 1) of aliased System.Secondary_Stack.SS_Stack (System.Parameters.Runtime_Default_Sec_Stack_Size);

   Local_Priority_Specific_Dispatching : constant String := "";
   Local_Interrupt_States : constant String := "";

   Is_Elaborated : Boolean := False;

   procedure adafinal is
      procedure s_stalib_adafinal;
      pragma Import (C, s_stalib_adafinal, "system__standard_library__adafinal");

      procedure Runtime_Finalize;
      pragma Import (C, Runtime_Finalize, "__gnat_runtime_finalize");

   begin
      if not Is_Elaborated then
         return;
      end if;
      Is_Elaborated := False;
      Runtime_Finalize;
      s_stalib_adafinal;
   end adafinal;

   procedure adainit is
      Main_Priority : Integer;
      pragma Import (C, Main_Priority, "__gl_main_priority");
      Time_Slice_Value : Integer;
      pragma Import (C, Time_Slice_Value, "__gl_time_slice_val");
      WC_Encoding : Character;
      pragma Import (C, WC_Encoding, "__gl_wc_encoding");
      Locking_Policy : Character;
      pragma Import (C, Locking_Policy, "__gl_locking_policy");
      Queuing_Policy : Character;
      pragma Import (C, Queuing_Policy, "__gl_queuing_policy");
      Task_Dispatching_Policy : Character;
      pragma Import (C, Task_Dispatching_Policy, "__gl_task_dispatching_policy");
      Priority_Specific_Dispatching : System.Address;
      pragma Import (C, Priority_Specific_Dispatching, "__gl_priority_specific_dispatching");
      Num_Specific_Dispatching : Integer;
      pragma Import (C, Num_Specific_Dispatching, "__gl_num_specific_dispatching");
      Main_CPU : Integer;
      pragma Import (C, Main_CPU, "__gl_main_cpu");
      Interrupt_States : System.Address;
      pragma Import (C, Interrupt_States, "__gl_interrupt_states");
      Num_Interrupt_States : Integer;
      pragma Import (C, Num_Interrupt_States, "__gl_num_interrupt_states");
      Unreserve_All_Interrupts : Integer;
      pragma Import (C, Unreserve_All_Interrupts, "__gl_unreserve_all_interrupts");
      Detect_Blocking : Integer;
      pragma Import (C, Detect_Blocking, "__gl_detect_blocking");
      Default_Stack_Size : Integer;
      pragma Import (C, Default_Stack_Size, "__gl_default_stack_size");
      Default_Secondary_Stack_Size : System.Parameters.Size_Type;
      pragma Import (C, Default_Secondary_Stack_Size, "__gnat_default_ss_size");
      Leap_Seconds_Support : Integer;
      pragma Import (C, Leap_Seconds_Support, "__gl_leap_seconds_support");
      Bind_Env_Addr : System.Address;
      pragma Import (C, Bind_Env_Addr, "__gl_bind_env_addr");

      procedure Runtime_Initialize (Install_Handler : Integer);
      pragma Import (C, Runtime_Initialize, "__gnat_runtime_initialize");
      Binder_Sec_Stacks_Count : Natural;
      pragma Import (Ada, Binder_Sec_Stacks_Count, "__gnat_binder_ss_count");
      Default_Sized_SS_Pool : System.Address;
      pragma Import (Ada, Default_Sized_SS_Pool, "__gnat_default_ss_pool");

   begin
      if Is_Elaborated then
         return;
      end if;
      Is_Elaborated := True;
      Main_Priority := -1;
      Time_Slice_Value := 0;
      WC_Encoding := 'b';
      Locking_Policy := 'C';
      Queuing_Policy := ' ';
      Task_Dispatching_Policy := 'F';
      System.Restrictions.Run_Time_Restrictions :=
        (Set =>
          (False, True, True, False, False, False, False, True, 
           False, False, False, False, False, False, False, True, 
           True, False, False, False, False, False, True, False, 
           False, False, False, False, False, False, False, False, 
           True, True, False, False, True, True, False, False, 
           False, True, False, False, False, False, True, False, 
           True, True, False, False, False, False, True, True, 
           True, True, True, False, True, False, False, False, 
           False, False, False, False, False, False, False, False, 
           False, False, False, False, False, True, False, False, 
           False, False, False, False, False, False, True, True, 
           False, True, False, False),
         Value => (0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
         Violated =>
          (False, False, False, False, True, True, False, False, 
           False, False, False, True, True, True, True, False, 
           False, False, False, False, True, True, False, True, 
           True, False, True, True, False, True, False, False, 
           False, False, False, True, False, False, True, False, 
           False, False, True, True, False, False, False, True, 
           False, False, False, True, False, False, False, False, 
           False, False, False, False, False, True, True, True, 
           False, False, True, False, True, True, True, False, 
           True, True, False, False, True, True, True, False, 
           False, True, False, False, False, True, False, False, 
           False, False, True, False),
         Count => (0, 0, 0, 1, 0, 0, 0, 0, 4, 0),
         Unknown => (False, False, False, False, False, False, False, False, True, False));
      Priority_Specific_Dispatching :=
        Local_Priority_Specific_Dispatching'Address;
      Num_Specific_Dispatching := 0;
      Main_CPU := -1;
      Interrupt_States := Local_Interrupt_States'Address;
      Num_Interrupt_States := 0;
      Unreserve_All_Interrupts := 0;
      Detect_Blocking := 1;
      Default_Stack_Size := -1;
      Leap_Seconds_Support := 0;

      ada_main'Elab_Body;
      Default_Secondary_Stack_Size := System.Parameters.Runtime_Default_Sec_Stack_Size;
      Binder_Sec_Stacks_Count := 1;
      Default_Sized_SS_Pool := Sec_Default_Sized_Stacks'Address;

      Runtime_Initialize (1);

      System.Soft_Links'Elab_Spec;
      System.Exception_Table'Elab_Body;
      E073 := E073 + 1;
      Ada.Tags'Elab_Body;
      E005 := E005 + 1;
      System.Bb.Timing_Events'Elab_Spec;
      E061 := E061 + 1;
      E075 := E075 + 1;
      Ada.Streams'Elab_Spec;
      E118 := E118 + 1;
      System.Finalization_Root'Elab_Spec;
      E126 := E126 + 1;
      Ada.Finalization'Elab_Spec;
      E124 := E124 + 1;
      System.Storage_Pools'Elab_Spec;
      E128 := E128 + 1;
      System.Finalization_Masters'Elab_Spec;
      System.Finalization_Masters'Elab_Body;
      E121 := E121 + 1;
      Ada.Real_Time'Elab_Body;
      E145 := E145 + 1;
      System.Pool_Global'Elab_Spec;
      E130 := E130 + 1;
      System.Tasking.Protected_Objects'Elab_Body;
      E195 := E195 + 1;
      System.Tasking.Protected_Objects.Multiprocessors'Elab_Body;
      E201 := E201 + 1;
      System.Tasking.Restricted.Stages'Elab_Body;
      E214 := E214 + 1;
      E140 := E140 + 1;
      E233 := E233 + 1;
      HAL.AUDIO'ELAB_SPEC;
      E219 := E219 + 1;
      HAL.BLOCK_DRIVERS'ELAB_SPEC;
      E242 := E242 + 1;
      HAL.GPIO'ELAB_SPEC;
      E141 := E141 + 1;
      HAL.I2C'ELAB_SPEC;
      E191 := E191 + 1;
      HAL.REAL_TIME_CLOCK'ELAB_SPEC;
      E223 := E223 + 1;
      HAL.SDMMC'ELAB_SPEC;
      E237 := E237 + 1;
      HAL.SPI'ELAB_SPEC;
      E116 := E116 + 1;
      HAL.TIME'ELAB_SPEC;
      E133 := E133 + 1;
      CS43L22'ELAB_SPEC;
      CS43L22'ELAB_BODY;
      E256 := E256 + 1;
      HAL.UART'ELAB_SPEC;
      E253 := E253 + 1;
      Ili9341_Extended'Elab_Spec;
      Ili9341_Extended'Elab_Body;
      E135 := E135 + 1;
      LIS3DSH'ELAB_SPEC;
      LIS3DSH'ELAB_BODY;
      E152 := E152 + 1;
      LIS3DSH.SPI'ELAB_SPEC;
      LIS3DSH.SPI'ELAB_BODY;
      E258 := E258 + 1;
      Ravenscar_Time'Elab_Spec;
      Ravenscar_Time'Elab_Body;
      E143 := E143 + 1;
      E235 := E235 + 1;
      STM32.ADC'ELAB_SPEC;
      E161 := E161 + 1;
      E167 := E167 + 1;
      E207 := E207 + 1;
      E180 := E180 + 1;
      E225 := E225 + 1;
      STM32.RTC'ELAB_SPEC;
      STM32.RTC'ELAB_BODY;
      E222 := E222 + 1;
      STM32.SPI'ELAB_SPEC;
      STM32.SPI'ELAB_BODY;
      E245 := E245 + 1;
      STM32.SPI.DMA'ELAB_SPEC;
      STM32.SPI.DMA'ELAB_BODY;
      E247 := E247 + 1;
      STM32.I2C'ELAB_SPEC;
      STM32.USARTS'ELAB_SPEC;
      STM32.I2S'ELAB_SPEC;
      STM32.I2C.DMA'ELAB_SPEC;
      STM32.GPIO'ELAB_SPEC;
      STM32.SDMMC'ELAB_SPEC;
      E241 := E241 + 1;
      STM32.GPIO'ELAB_BODY;
      E173 := E173 + 1;
      STM32.DEVICE'ELAB_SPEC;
      E156 := E156 + 1;
      STM32.SDMMC'ELAB_BODY;
      E230 := E230 + 1;
      STM32.I2S'ELAB_BODY;
      E218 := E218 + 1;
      STM32.I2C.DMA'ELAB_BODY;
      E193 := E193 + 1;
      STM32.I2C'ELAB_BODY;
      E187 := E187 + 1;
      E178 := E178 + 1;
      STM32.USARTS'ELAB_BODY;
      E251 := E251 + 1;
      E154 := E154 + 1;
      STM32.BOARD'ELAB_SPEC;
      E150 := E150 + 1;
      STM32.USER_BUTTON'ELAB_BODY;
      E260 := E260 + 1;
   end adainit;

   procedure Ada_Main_Program;
   pragma Import (Ada, Ada_Main_Program, "_ada_main");

   procedure main is
      procedure Initialize (Addr : System.Address);
      pragma Import (C, Initialize, "__gnat_initialize");

      procedure Finalize;
      pragma Import (C, Finalize, "__gnat_finalize");
      SEH : aliased array (1 .. 2) of Integer;

      Ensure_Reference : aliased System.Address := Ada_Main_Program_Name'Address;
      pragma Volatile (Ensure_Reference);

   begin
      Initialize (SEH'Address);
      adainit;
      Ada_Main_Program;
      adafinal;
      Finalize;
   end;

--  BEGIN Object file/option list
   --   C:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\screen test\obj\bmp_fonts.o
   --   C:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\screen test\obj\ili9341_extended.o
   --   C:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\screen test\obj\main.o
   --   -LC:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\screen test\obj\
   --   -LC:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\screen test\obj\
   --   -LC:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\shared\common\
   --   -LC:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\boards\stm32f407_discovery\obj\full_lib_Debug\
   --   -LC:\gnat\2018-arm-elf\arm-eabi\lib\gnat\ravenscar-full-stm32f4\adalib\
   --   -static
   --   -lgnarl
   --   -lgnat
--  END Object file/option list   

end ada_main;
