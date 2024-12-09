module MyDeployment {

  # ----------------------------------------------------------------------
  # Symbolic constants for port numbers
  # ----------------------------------------------------------------------

  enum Ports_RateGroups {
    rateGroup1
    rateGroup2
    rateGroup3
  }

  topology MyDeployment {

    # ----------------------------------------------------------------------
    # Instances used in the topology
    # ----------------------------------------------------------------------

    instance $health
    instance blockDrv
    instance tlmSend
    instance cmdDisp
    instance cmdSeq
    instance comQueue
    instance deframer
    instance eventLogger
    instance fatalAdapter
    instance fatalHandler
    instance fileDownlink
    instance fileManager
    instance fileUplink
    instance bufferManager
    instance framer
    instance posixTime
    instance prmDb
    instance rateGroup1
    instance rateGroup2
    instance rateGroup3
    instance rateGroupDriver
    instance textLogger
    instance systemResources

    instance radioResetPin
    instance radioChipSelectPin
    instance radioInterruptPin
    instance spiDriver
    instance radio

    # ----------------------------------------------------------------------
    # Pattern graph specifiers
    # ----------------------------------------------------------------------

    command connections instance cmdDisp

    event connections instance eventLogger

    param connections instance prmDb

    telemetry connections instance tlmSend

    text event connections instance textLogger

    time connections instance posixTime

    health connections instance $health

    # ----------------------------------------------------------------------
    # Direct graph specifiers
    # ----------------------------------------------------------------------

    connections Downlink {

      eventLogger.PktSend -> comQueue.comQueueIn[0]
      tlmSend.PktSend -> comQueue.comQueueIn[1]
      fileDownlink.bufferSendOut -> comQueue.buffQueueIn[0]

      comQueue.comQueueSend -> framer.comIn
      comQueue.buffQueueSend -> framer.bufferIn

      framer.framedAllocate -> bufferManager.bufferGetCallee
      framer.framedOut -> radio.comDataIn
      framer.bufferDeallocate -> fileDownlink.bufferReturn

      radio.deallocate -> bufferManager.bufferSendIn

      radio.comStatus -> framer.comStatusIn
      framer.comStatusOut -> comQueue.comStatusIn

    }

    connections FaultProtection {
      eventLogger.FatalAnnounce -> fatalHandler.FatalReceive
    }

    connections RateGroups {
      # Block driver
      blockDrv.CycleOut -> rateGroupDriver.CycleIn

      # Rate group 1 (100 ms)
      rateGroupDriver.CycleOut[Ports_RateGroups.rateGroup1] -> rateGroup1.CycleIn
      rateGroup1.RateGroupMemberOut[0] -> radio.run

      # Rate group 2
      rateGroupDriver.CycleOut[Ports_RateGroups.rateGroup2] -> rateGroup2.CycleIn
      rateGroup2.RateGroupMemberOut[0] -> cmdSeq.schedIn
      rateGroup2.RateGroupMemberOut[1] -> tlmSend.Run
      rateGroup2.RateGroupMemberOut[2] -> fileDownlink.Run
      rateGroup2.RateGroupMemberOut[3] -> systemResources.run

      # Rate group 3
      rateGroupDriver.CycleOut[Ports_RateGroups.rateGroup3] -> rateGroup3.CycleIn
      rateGroup3.RateGroupMemberOut[0] -> $health.Run
      rateGroup3.RateGroupMemberOut[1] -> blockDrv.Sched
      rateGroup3.RateGroupMemberOut[2] -> bufferManager.schedIn
    }

    connections Sequencer {
      cmdSeq.comCmdOut -> cmdDisp.seqCmdBuff
      cmdDisp.seqCmdStatus -> cmdSeq.cmdResponseIn
    }

    connections Uplink {

      radio.allocate -> bufferManager.bufferGetCallee
      radio.comDataOut -> deframer.framedIn

      deframer.framedDeallocate -> bufferManager.bufferSendIn
      deframer.comOut -> cmdDisp.seqCmdBuff

      cmdDisp.seqCmdStatus -> deframer.cmdResponseIn

      deframer.bufferAllocate -> bufferManager.bufferGetCallee
      deframer.bufferOut -> fileUplink.bufferSendIn
      deframer.bufferDeallocate -> bufferManager.bufferSendIn
      fileUplink.bufferSendOut -> bufferManager.bufferSendIn
    }

    connections MyDeployment {
      # Add here connections to user-defined components
      radio.radioReset -> radioResetPin.gpioWrite
      radio.radioChipSelect -> radioChipSelectPin.gpioWrite
      radioInterruptPin.gpioInterrupt -> radio.radioInterrupt
      radio.spiReadWrite -> spiDriver.SpiReadWrite
    }

  }

}
