pragma Warnings (Off);
pragma Ada_95;
pragma Source_File_Name (ada_main, Spec_File_Name => "b__main.ads");
pragma Source_File_Name (ada_main, Body_File_Name => "b__main.adb");
pragma Suppress (Overflow_Check);

with System.Restrictions;

package body ada_main is

   E009 : Short_Integer; pragma Import (Ada, E009, "system__soft_links_E");
   E045 : Short_Integer; pragma Import (Ada, E045, "system__exception_table_E");
   E088 : Short_Integer; pragma Import (Ada, E088, "ada__tags_E");
   E102 : Short_Integer; pragma Import (Ada, E102, "system__bb__timing_events_E");
   E140 : Short_Integer; pragma Import (Ada, E140, "ada__streams_E");
   E148 : Short_Integer; pragma Import (Ada, E148, "system__finalization_root_E");
   E146 : Short_Integer; pragma Import (Ada, E146, "ada__finalization_E");
   E150 : Short_Integer; pragma Import (Ada, E150, "system__storage_pools_E");
   E143 : Short_Integer; pragma Import (Ada, E143, "system__finalization_masters_E");
   E135 : Short_Integer; pragma Import (Ada, E135, "ada__real_time_E");
   E152 : Short_Integer; pragma Import (Ada, E152, "system__pool_global_E");
   E118 : Short_Integer; pragma Import (Ada, E118, "system__tasking__protected_objects_E");
   E122 : Short_Integer; pragma Import (Ada, E122, "system__tasking__protected_objects__multiprocessors_E");
   E127 : Short_Integer; pragma Import (Ada, E127, "system__tasking__restricted__stages_E");
   E268 : Short_Integer; pragma Import (Ada, E268, "bmp_fonts_E");
   E234 : Short_Integer; pragma Import (Ada, E234, "cortex_m__cache_E");
   E220 : Short_Integer; pragma Import (Ada, E220, "hal__audio_E");
   E243 : Short_Integer; pragma Import (Ada, E243, "hal__block_drivers_E");
   E193 : Short_Integer; pragma Import (Ada, E193, "hal__gpio_E");
   E201 : Short_Integer; pragma Import (Ada, E201, "hal__i2c_E");
   E224 : Short_Integer; pragma Import (Ada, E224, "hal__real_time_clock_E");
   E238 : Short_Integer; pragma Import (Ada, E238, "hal__sdmmc_E");
   E138 : Short_Integer; pragma Import (Ada, E138, "hal__spi_E");
   E161 : Short_Integer; pragma Import (Ada, E161, "hal__time_E");
   E257 : Short_Integer; pragma Import (Ada, E257, "cs43l22_E");
   E254 : Short_Integer; pragma Import (Ada, E254, "hal__uart_E");
   E265 : Short_Integer; pragma Import (Ada, E265, "ili9341_extended_E");
   E156 : Short_Integer; pragma Import (Ada, E156, "lis3dsh_E");
   E259 : Short_Integer; pragma Import (Ada, E259, "lis3dsh__spi_E");
   E160 : Short_Integer; pragma Import (Ada, E160, "ravenscar_time_E");
   E236 : Short_Integer; pragma Import (Ada, E236, "sdmmc_init_E");
   E170 : Short_Integer; pragma Import (Ada, E170, "stm32__adc_E");
   E176 : Short_Integer; pragma Import (Ada, E176, "stm32__dac_E");
   E213 : Short_Integer; pragma Import (Ada, E213, "stm32__dma__interrupts_E");
   E189 : Short_Integer; pragma Import (Ada, E189, "stm32__exti_E");
   E226 : Short_Integer; pragma Import (Ada, E226, "stm32__power_control_E");
   E223 : Short_Integer; pragma Import (Ada, E223, "stm32__rtc_E");
   E246 : Short_Integer; pragma Import (Ada, E246, "stm32__spi_E");
   E248 : Short_Integer; pragma Import (Ada, E248, "stm32__spi__dma_E");
   E197 : Short_Integer; pragma Import (Ada, E197, "stm32__i2c_E");
   E252 : Short_Integer; pragma Import (Ada, E252, "stm32__usarts_E");
   E242 : Short_Integer; pragma Import (Ada, E242, "stm32__sdmmc_interrupt_E");
   E219 : Short_Integer; pragma Import (Ada, E219, "stm32__i2s_E");
   E203 : Short_Integer; pragma Import (Ada, E203, "stm32__i2c__dma_E");
   E182 : Short_Integer; pragma Import (Ada, E182, "stm32__gpio_E");
   E231 : Short_Integer; pragma Import (Ada, E231, "stm32__sdmmc_E");
   E187 : Short_Integer; pragma Import (Ada, E187, "stm32__syscfg_E");
   E165 : Short_Integer; pragma Import (Ada, E165, "stm32__device_E");
   E163 : Short_Integer; pragma Import (Ada, E163, "stm32__setup_E");
   E133 : Short_Integer; pragma Import (Ada, E133, "stm32__board_E");
   E129 : Short_Integer; pragma Import (Ada, E129, "apds9960_gesture_E");
   E263 : Short_Integer; pragma Import (Ada, E263, "apds9960_interrupts_E");
   E270 : Short_Integer; pragma Import (Ada, E270, "last_chance_handler_E");
   E272 : Short_Integer; pragma Import (Ada, E272, "stm32__user_button_E");

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
         Count => (0, 0, 0, 1, 0, 0, 0, 0, 5, 0),
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
      E045 := E045 + 1;
      Ada.Tags'Elab_Body;
      E088 := E088 + 1;
      System.Bb.Timing_Events'Elab_Spec;
      E009 := E009 + 1;
      E102 := E102 + 1;
      Ada.Streams'Elab_Spec;
      E140 := E140 + 1;
      System.Finalization_Root'Elab_Spec;
      E148 := E148 + 1;
      Ada.Finalization'Elab_Spec;
      E146 := E146 + 1;
      System.Storage_Pools'Elab_Spec;
      E150 := E150 + 1;
      System.Finalization_Masters'Elab_Spec;
      System.Finalization_Masters'Elab_Body;
      E143 := E143 + 1;
      Ada.Real_Time'Elab_Body;
      E135 := E135 + 1;
      System.Pool_Global'Elab_Spec;
      E152 := E152 + 1;
      System.Tasking.Protected_Objects'Elab_Body;
      E118 := E118 + 1;
      System.Tasking.Protected_Objects.Multiprocessors'Elab_Body;
      E122 := E122 + 1;
      System.Tasking.Restricted.Stages'Elab_Body;
      E127 := E127 + 1;
      E268 := E268 + 1;
      E234 := E234 + 1;
      HAL.AUDIO'ELAB_SPEC;
      E220 := E220 + 1;
      HAL.BLOCK_DRIVERS'ELAB_SPEC;
      E243 := E243 + 1;
      HAL.GPIO'ELAB_SPEC;
      E193 := E193 + 1;
      HAL.I2C'ELAB_SPEC;
      E201 := E201 + 1;
      HAL.REAL_TIME_CLOCK'ELAB_SPEC;
      E224 := E224 + 1;
      HAL.SDMMC'ELAB_SPEC;
      E238 := E238 + 1;
      HAL.SPI'ELAB_SPEC;
      E138 := E138 + 1;
      HAL.TIME'ELAB_SPEC;
      E161 := E161 + 1;
      CS43L22'ELAB_SPEC;
      CS43L22'ELAB_BODY;
      E257 := E257 + 1;
      HAL.UART'ELAB_SPEC;
      E254 := E254 + 1;
      Ili9341_Extended'Elab_Spec;
      Ili9341_Extended'Elab_Body;
      E265 := E265 + 1;
      LIS3DSH'ELAB_SPEC;
      LIS3DSH'ELAB_BODY;
      E156 := E156 + 1;
      LIS3DSH.SPI'ELAB_SPEC;
      LIS3DSH.SPI'ELAB_BODY;
      E259 := E259 + 1;
      Ravenscar_Time'Elab_Spec;
      Ravenscar_Time'Elab_Body;
      E160 := E160 + 1;
      E236 := E236 + 1;
      STM32.ADC'ELAB_SPEC;
      E170 := E170 + 1;
      E176 := E176 + 1;
      E213 := E213 + 1;
      E189 := E189 + 1;
      E226 := E226 + 1;
      STM32.RTC'ELAB_SPEC;
      STM32.RTC'ELAB_BODY;
      E223 := E223 + 1;
      STM32.SPI'ELAB_SPEC;
      STM32.SPI'ELAB_BODY;
      E246 := E246 + 1;
      STM32.SPI.DMA'ELAB_SPEC;
      STM32.SPI.DMA'ELAB_BODY;
      E248 := E248 + 1;
      STM32.I2C'ELAB_SPEC;
      STM32.USARTS'ELAB_SPEC;
      STM32.I2S'ELAB_SPEC;
      STM32.I2C.DMA'ELAB_SPEC;
      STM32.GPIO'ELAB_SPEC;
      STM32.SDMMC'ELAB_SPEC;
      E242 := E242 + 1;
      STM32.GPIO'ELAB_BODY;
      E182 := E182 + 1;
      STM32.DEVICE'ELAB_SPEC;
      E165 := E165 + 1;
      STM32.SDMMC'ELAB_BODY;
      E231 := E231 + 1;
      STM32.I2S'ELAB_BODY;
      E219 := E219 + 1;
      STM32.I2C.DMA'ELAB_BODY;
      E203 := E203 + 1;
      STM32.I2C'ELAB_BODY;
      E197 := E197 + 1;
      E187 := E187 + 1;
      STM32.USARTS'ELAB_BODY;
      E252 := E252 + 1;
      E163 := E163 + 1;
      STM32.BOARD'ELAB_SPEC;
      E133 := E133 + 1;
      apds9960_gesture'elab_spec;
      apds9960_gesture'elab_body;
      E129 := E129 + 1;
      Apds9960_Interrupts'Elab_Spec;
      E263 := E263 + 1;
      E270 := E270 + 1;
      STM32.USER_BUTTON'ELAB_BODY;
      E272 := E272 + 1;
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
   --   C:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\gesture_test\obj\bmp_fonts.o
   --   C:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\gesture_test\obj\ili9341_extended.o
   --   C:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\gesture_test\obj\apds9960_gesture.o
   --   C:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\gesture_test\obj\apds9960_interrupts.o
   --   C:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\gesture_test\obj\last_chance_handler.o
   --   C:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\gesture_test\obj\main.o
   --   -LC:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\gesture_test\obj\
   --   -LC:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\STM32F4_DISCO\gesture_test\obj\
   --   -LC:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\examples\shared\common\
   --   -LC:\Users\DELL\Downloads\ADA_\Ada_Drivers_Library\boards\stm32f407_discovery\obj\full_lib_Debug\
   --   -LC:\gnat\2018-arm-elf\arm-eabi\lib\gnat\ravenscar-full-stm32f4\adalib\
   --   -static
   --   -lgnarl
   --   -lgnat
--  END Object file/option list   

end ada_main;
