global with sharing class TemperatureListController {
  // ApexPages.StandardSetController must be instantiated 
    
  // for standard list controllers 
    
    public ApexPages.StandardSetController setCon {
        get {
            if(setCon == null) {
                setCon = new ApexPages.StandardSetController(Database.getQueryLocator(
                      [select name,temp__c,createdDate  from temperature__c order by createdDate desc]));
            }
            return setCon;
        }
        set;
    }

    // Initialize setCon and return a list of records 
    
    public List<temperature__c> getTemperatures() {
         return (List<temperature__c>) setCon.getRecords();
    }
    
    @RemoteAction
    global static temperature__c getLatestTemperature() 
    {
        List<temperature__c> temps = [SELECT name, temp__c, tempid__c, CreatedDate FROM temperature__c order by createdDate desc];
        return temps.size() > 0 ? temps[0] : null;
    }    
}

