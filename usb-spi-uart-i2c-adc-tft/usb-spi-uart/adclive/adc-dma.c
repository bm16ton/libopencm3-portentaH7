volatile static uint16_t adc_res[17];

void adc1_setup(void) {

    static uint8_t channel_seq[16];

    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);

    adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2);

    //adc_power_off(ADC1);

    adc_enable_scan_mode(ADC1);
    adc_set_continuous_conversion_mode(ADC1);
    adc_disable_discontinuous_mode_regular(ADC1);

    //adc_enable_temperature_sensor();

    adc_enable_external_trigger_regular(ADC1, ADC_CR2_SWSTART, ADC_CR2_EXTEN_DISABLED);
    adc_set_right_aligned(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28CYC);
    adc_set_resolution(ADC1, 8);


    channel_seq[0] = ADC_CHANNEL0;
    channel_seq[1] = ADC_CHANNEL1;

    adc_set_multi_mode(ADC_CCR_MULTI_INDEPENDENT);

    adc_set_regular_sequence(ADC1, 1, channel_seq);
    adc_enable_dma(ADC1);

    adc_power_on(ADC1);

    adc_start_conversion_regular(ADC1);
}

/*** DMA2 ***/
static void dma2_setup(void) {

    dma_stream_reset(DMA2, DMA_STREAM4);
    dma_set_priority(DMA2, DMA_STREAM4, DMA_SxCR_PL_LOW);

    dma_set_memory_size(DMA2, DMA_STREAM4, DMA_SxCR_MSIZE_16BIT);
    dma_set_peripheral_size(DMA2, DMA_STREAM4, DMA_SxCR_PSIZE_16BIT);

    dma_enable_memory_increment_mode(DMA2, DMA_STREAM4);
    dma_enable_circular_mode(DMA2, DMA_STREAM4);

    dma_set_transfer_mode(DMA2, DMA_STREAM4, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_set_peripheral_address(DMA2, DMA_STREAM4, (uint32_t) & ADC_DR(ADC1));
    dma_set_memory_address(DMA2, DMA_STREAM4, (uint32_t) &adc_res);
    dma_set_number_of_data(DMA2, DMA_STREAM4, 1);


    dma_channel_select(DMA2, DMA_STREAM4, DMA_SxCR_CHSEL_0);

    //nvic_enable_irq(NVIC_DMA2_STREAM4_IRQ);
    //dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM4);
    dma_enable_stream(DMA2, DMA_STREAM4);
}

void dma2_stream4_isr(void) {
    //dma_clear_interrupt_flags(DMA2, DMA_CHANNEL4, DMA_IFCR_CGIF1);
}

void dma2_stream0_isr(void) {
    //dma_clear_interrupt_flags(DMA2, DMA_CHANNEL4, DMA_IFCR_CGIF1);
}


float get_mcu_temp(void) {
    float temp;
    temp = adc_res[0];  //((((adc_res[2] * 3300.0f) / 0xFFF) / 1000.0f) - 0.760f) / .0025f;
    return temp;
}